#include <V2Buttons.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2MIDI.h>

V2DEVICE_METADATA("com.versioduo.radio", 2, "versioduo:samd:radio");

namespace {
  V2LED::WS2812        Led(2, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
  V2MIDI::SerialDevice MIDISerial(&SerialMIDI);
  V2MIDI::SerialDevice MIDIRadio(&SerialRadio);
  V2Device*            device{};

  // 433MHz RF Module EBYTE E62-433T20S
  class V2E62 {
  public:
    struct Header {
      enum class Command : uint8_t {
        Base             = 0xc0,
        ConfigWriteFlash = Base,
        ConfigRead,
        ConfigWrite,
        Version,
        Restart,
        Signal,
      };

      static auto write(Uart& s, Command c) {
        std::array bytes{uint8_t(c), uint8_t(c), uint8_t(c)};
        s.write(bytes.data(), bytes.size());
      }
    };

    struct Config {
      Header::Command command{Header::Command::Base};
      struct FHS {
        uint8_t id{1};
        uint8_t channels{10};

        auto operator<=>(const FHS&) const = default;
      } fhs;
      struct Speed {
        uint8_t air : 3 {2};
        uint8_t uart : 3 {3};
        uint8_t parity : 2 {};

        auto operator<=>(const Speed&) const = default;

        auto airKbps() const -> uint8_t {
          return 1 << (air + 4);
        }

        static auto airKbps(uint8_t s) -> uint8_t {
          return std::countr_zero(s) - 4;
        }

        auto uartBps() const -> uint32_t {
          return std::array{1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200}.at(uart);
        }

        static auto uartBps(uint32_t s) -> uint8_t {
          switch (s) {
            case 1200:
              return 0;

            case 2400:
              return 1;

            case 4800:
              return 2;

            case 9600:
              return 3;

            case 19200:
              return 4;

            case 38400:
              return 5;

            case 57600:
              return 6;

            case 115200:
              return 7;

            default:
              std::abort();
          }
        }

      } speed;
      uint8_t channel{10};
      struct Options {
        uint8_t power : 2 {};
        uint8_t fec : 1 {1};
        uint8_t unused0 : 3 {};
        uint8_t drive : 1 {1};
        uint8_t unused1 : 1 {};

        auto operator<=>(const Options&) const = default;
      } options;

      auto operator<=>(const Config&) const = default;

      static auto frequency(uint8_t c) -> float {
        return 425.f + (float(c) * 0.5f);
      }

      auto data() const -> const uint8_t* {
        return (const uint8_t*)this;
      }

      auto dump(Stream& s) const {
        s.printf("Channel:      %d (%s MHz)\n", channel, String(frequency(channel), 1).c_str());
        s.printf("FHS Id:       %d\n", fhs.id, fhs.channels);
        s.printf("FHS Channels: %d (%s-%s MHz)\n",
                 fhs.channels,
                 String(frequency(channel), 1).c_str(),
                 String(frequency(channel + fhs.channels), 1).c_str());
        s.printf("Power:        %d dBm\n", std::array{20, 17, 13, 10}[options.power]);
        s.printf("AirRate:      %d k\n", speed.airKbps());
        s.printf("FEC:          %s\n", options.fec ? "enabled" : "disabled");
        s.printf("UART:         %d %s\n", speed.uartBps(), std::array{"8N1", "8O1", "8E1", "8N1"}[speed.parity]);
        s.printf("IO Mode:      %s\n", options.drive ? "push-pull" : "open-drain");
      }
    };
    static_assert(sizeof(Config) == 6);

    // ITU region 1 ISM band: 433.050 MHz - 434.790 MHz
    Config config{
      .command{Header::Command::ConfigWrite},
      .fhs{
        .id{0xd2},
        .channels{3},
      },
      .speed{
        .air{Config::Speed::airKbps(64)},
        .uart{Config::Speed::uartBps(115200)},
      },
      .channel{16},
    };

    struct Signal {
      Header::Command command{Header::Command::Signal};
      uint8_t         rssi{};
      uint8_t         noise{};
    };

    auto begin() {
      pinMode(PIN_RADIO_RESET, OUTPUT);
      digitalWrite(PIN_RADIO_RESET, HIGH);

      // Config = high, Transmit = low.
      pinMode(PIN_RADIO_MODE, OUTPUT);
      digitalWrite(PIN_RADIO_MODE, HIGH);

      pinMode(PIN_RADIO_LOCK, INPUT);
      pinMode(PIN_RADIO_AUX, INPUT);

      SerialRadio.begin(9600);
      SerialRadio.setTimeout(1);
    }

    auto reset() {
      _state = State::Restart;
      _debug = false;
      _link  = {};
    }

    auto loop() {
      switch (_state) {
        case State::ModeConfig: {
          Led.reset();
          digitalWrite(PIN_RADIO_MODE, HIGH);
          delay(100);
          SerialRadio.end();
          SerialRadio.begin(9600);
          SerialRadio.write(config.data(), sizeof(Config));
          delay(100);
          _state = State::CheckConfig;
        } break;

        case State::CheckConfig: {
          Header::write(SerialRadio, Header::Command::ConfigRead);
          delay(100);
          if (Config c; size_t(SerialRadio.available()) >= sizeof(c)) {
            SerialRadio.readBytes((uint8_t*)&c, sizeof(c));
            if (c != config) {
              _state = State::ModeConfig;
              break;
            }

            if (_debug) {
              _debug = false;
              c.dump(Serial);
              Header::write(SerialRadio, Header::Command::Signal);
              delay(100);
              if (Signal signal; size_t(SerialRadio.available()) >= sizeof(signal)) {
                SerialRadio.readBytes((uint8_t*)&signal, sizeof(signal));
                Serial.printf("Signal RSSI:  %d/%d\n", signal.rssi, signal.noise);
              }
            }
          }
          _state = State::ModeTransmit;
          break;
        }

        case State::ModeTransmit:
          SerialRadio.end();
          SerialRadio.begin(config.speed.uartBps());
          digitalWrite(PIN_RADIO_MODE, LOW);
          delay(100);
          _state = State::Running;
          break;

        case State::Running: {
          if (!online())
            break;

          if (!MIDIRadio.receive(&_midi))
            break;

          device->led.flash(0.03, 0.3);
          device->usb.midi.send(&_midi);
          MIDISerial.send(&_midi);
          break;
        }

        case State::Restart:
          digitalWrite(PIN_RADIO_RESET, LOW);
          delay(100);
          digitalWrite(PIN_RADIO_RESET, HIGH);
          reset();
          _state = State::ModeConfig;
          break;
      }
    }

    auto online() -> bool {
      if (V2Base::getUsecSince(_link.usec) < 200 * 1000)
        return true;
      _link.usec = V2Base::getUsec();

      if (auto online{digitalRead(PIN_RADIO_LOCK) == HIGH}; online && _link.online)
        return true;
      else
        _link.online = online;

      if (_link.online) {
        Led.setHSV(V2Colour::Orange, 1, 0.35);
        _link.resetUsec = 0;

      } else {
        Led.setHSV(V2Colour::Cyan, 1, 0.25);

        if (_link.resetUsec == 0)
          _link.resetUsec = V2Base::getUsec();

        if (V2Base::getUsecSince(_link.resetUsec) > 3 * 1000 * 1000) {
          reset();
          return false;
        }
      }

      return true;
    }

    auto send(V2MIDI::Packet& m) {
      device->led.flash(0.03, 0.3);
      MIDIRadio.send(&m);
    }

    auto debug() {
      _state = State::ModeConfig;
      _debug = true;
    }

  private:
    enum class State {
      ModeConfig,
      CheckConfig,
      ModeTransmit,
      Running,
      Restart,
    } _state{};
    bool _debug{};

    V2MIDI::Packet _midi;
    struct {
      bool     online{};
      uint32_t usec{};
      uint32_t resetUsec{};
    } _link;
  } Radio;

  class Device : public V2Device {
  public:
    Device() : V2Device() {
      metadata.description = "433MHz MIDI Bridge";
      metadata.vendor      = "Versio Duo";
      metadata.product     = "V2 radio";
      metadata.home        = "https://versioduo.com/#beat";

      usb.ports.standard = 0;

      system.download  = "https://versioduo.com/download";
      system.configure = "https://versioduo.com/configure";

      configuration = {.size{sizeof(Radio.config)}, .data{&Radio.config}};
      device        = this;
    }

  private:
    void handleReset() override {}

    void handleSystemReset() override {
      reset();
    }

    auto handlePacket(V2MIDI::Packet* m) -> void override {
      Radio.send(*m);
    }

    auto exportConfiguration(JsonObject json) -> void override {
      json["#channel"]  = "Channel 0 - 51 (425 – 450.5MHz)";
      json["channel"]   = Radio.config.channel;
      json["#id"]       = "Frequency Hopping / Spread Spectrum Id";
      json["id"]        = Radio.config.fhs.id;
      json["#channels"] = "1 - 51 Number of Frequency Hopping / Spread Spectrum Channels";
      json["channels"]  = Radio.config.fhs.channels;
    }

    auto importConfiguration(JsonObject json) -> void override {
      if (!json["channel"].isNull())
        Radio.config.channel = json["channel"];
      if (!json["id"].isNull())
        Radio.config.fhs.id = json["id"];
      if (!json["channels"].isNull())
        Radio.config.fhs.channels = json["channels"];
    }

    auto exportSettings(JsonArray json) -> void override {
      {
        JsonObject s{json.add<JsonObject>()};
        s["type"]  = "number";
        s["title"] = "Frequency";
        s["label"] = "Channel";
        s["min"]   = 0;
        s["max"]   = 51;
        s["path"]  = "channel";
      }
      {
        JsonObject s{json.add<JsonObject>()};
        s["type"]  = "number";
        s["title"] = "Spread Spectrum";
        s["label"] = "Id";
        s["min"]   = 0;
        s["max"]   = 255;
        s["path"]  = "id";
      }
      {
        JsonObject s{json.add<JsonObject>()};
        s["type"]  = "number";
        s["label"] = "Channels";
        s["min"]   = 1;
        s["max"]   = 51;
        s["path"]  = "channels";
      }
    }
  } Device;

  // Dispatch MIDI packets.
  class {
  public:
    auto loop() {
      if (Device.usb.midi.receive(&_midi))
        Device.dispatch(&Device.usb.midi, &_midi);

      if (MIDISerial.receive(&_midi))
        Radio.send(_midi);
    }

  private:
    V2MIDI::Packet _midi;
  } MIDI;

  class Button : public V2Buttons::Button {
  public:
    Button() : V2Buttons::Button(&config, PIN_BUTTON) {}

  private:
    const V2Buttons::Config config{.clickUsec{200 * 1000}, .holdUsec{500 * 1000}};

    auto handleClick(uint8_t count) -> void override {
      switch (count) {
        case 0:
          Radio.reset();
          Device.reset();
          break;

        case 1:
          Radio.debug();
          break;
      }
    }
  } Button;
}

auto setup() -> void {
  Serial.begin(9600);

  Led.begin();
  Led.setMaxBrightness(0.5);

  MIDIRadio.begin();
  MIDISerial.begin();
  Device.serial = &MIDISerial;

  Radio.begin();
  Radio.reset();

  Button.begin();
  Device.begin();
  Device.reset();
}

auto loop() -> void {
  Led.loop();
  Radio.loop();
  MIDI.loop();
  V2Buttons::loop();
  Device.loop();

  if (Device.idle())
    Device.sleep();
}
