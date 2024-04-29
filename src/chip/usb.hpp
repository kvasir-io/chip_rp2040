#pragma once

#include "chip/chip.hpp"
#include "kvasir/Util/StaticVector.hpp"

#include <array>
#include <charconv>
#include <cstdint>
#include <span>

namespace Kvasir { namespace USB {

    enum class DescriptorType : std::uint8_t {
        Device               = 0x01,
        Configuration        = 0x02,
        Interface            = 0x04,
        Endpoint             = 0x05,
        InterfaceAssociation = 0x0b,
        CS_Interface         = 0x24,
    };
    enum class DescriptorSubType : std::uint8_t {
        CDC_Header         = 0x00,
        CDC_CallManagement = 0x01,
        CDC_Union          = 0x06,
        CDC_ACM_Descriptor = 0x02
    };

    enum class DeviceClass : std::uint8_t {
        Communication = 0x02,
        CDC_Data      = 0x0A,
        Miscellaneous = 0xEF,
    };

    enum class ConfigurationAttributes : std::uint8_t {
        RemoteWakeup = 0x20,
        SelfePowered = 0x40,
        BusPowered   = 0x80
    };

    enum class EndpointTransferType : std::uint8_t {
        Control     = 0x0,
        Isochronous = 0x1,
        Bulk        = 0x02,
        Interrupt   = 0x03,
    };

    enum class EndpointDirection : std::uint8_t { Out = 0x00, In = 0x80 };

    constexpr std::uint8_t
    makeEndpointAddress(EndpointDirection direction, std::uint8_t endpointID) {
        return static_cast<std::uint8_t>(direction) | endpointID;
    }

    namespace detail {
        template<typename Child, DescriptorType DT>
        struct DescriptorBase {
            std::uint8_t   bLength{sizeof(Child)};
            DescriptorType bDescriptorType{DT};
        };
        template<typename Child, DescriptorSubType DST>
        struct InterfaceDescriptorBase : DescriptorBase<Child, DescriptorType::CS_Interface> {
            DescriptorSubType bDescriptorSubType{DST};
        };
    }   // namespace detail

    struct SetupPacket {
        enum class Direction : std::uint8_t { hostToDevice, deviceToHost };

        enum class Type : std::uint8_t { standard, classT, vendor, reserved };

        enum class Recipiant : std::uint8_t { device, interface, endpoint, other };

        enum class Request : std::uint8_t {
            getStatus     = 0,
            clearFeature  = 1,
            setFeature    = 3,
            setAddress    = 5,
            getDescriptor = 6,
            setDescriptor,
            getConfiguration,
            setConfiguration,
            getInterface,
            setInterface,
            setLineCoding       = 0x20,
            getLineCoding       = 0x21,
            setControlLineState = 0x22
        };

        enum class DescriptorType : std::uint8_t {
            device                    = 1,
            configuration             = 2,
            string                    = 3,
            interface                 = 4,
            endpoint                  = 5,
            deviceQualifier           = 6,
            otherSpeedConfiguration   = 7,
            interrfacePower           = 8,
            otg                       = 9,
            debug                     = 10,
            interfaceAssociation      = 11,
            security                  = 12,
            key                       = 13,
            encriptionType            = 14,
            bos                       = 15,
            deviceCapability          = 16,
            wirelessEndpointCompanion = 17
        };

        Direction direction() const { return static_cast<Direction>((bmRequestType & 0x80) >> 7); }
        Type      type() const { return static_cast<Type>((bmRequestType & 0x60) >> 5); }
        Recipiant recipiant() const { return static_cast<Recipiant>(bmRequestType & 0x1f); }
        DescriptorType descriptorType() const { return static_cast<DescriptorType>(wValue >> 8); }

        uint8_t  bmRequestType;
        Request  bRequest;
        uint16_t wValue;
        uint16_t wIndex;
        uint16_t wLength;
    };

    namespace Descriptors {
        struct Device : detail::DescriptorBase<Device, DescriptorType::Device> {
            std::uint16_t bcdUSB{0x0200};
            DeviceClass   bDeviceClass;
            DeviceClass   bDeviceSubClass;
            std::uint8_t  bDeviceProtocol;
            std::uint8_t  bMaxPacketSize0;
            std::uint16_t idVendor;
            std::uint16_t idProduct;
            std::uint16_t bcdDevice;
            std::uint8_t  iManufacturer;
            std::uint8_t  iProduct;
            std::uint8_t  iSerialNumber;
            std::uint8_t  bNumConfigurations;
        };

        struct [[gnu::packed]] Configuration
          : detail::DescriptorBase<Configuration, DescriptorType::Configuration> {
            std::uint16_t           wTotalLength;
            std::uint8_t            bNumInterfaces;
            std::uint8_t            bConfigurationValue;
            std::uint8_t            iConfiguration{};
            ConfigurationAttributes bmAttributes{ConfigurationAttributes::BusPowered};
            std::uint8_t            bMaxPower;
        };

        struct Interface : detail::DescriptorBase<Interface, DescriptorType::Interface> {
            std::uint8_t bInterfaceNumber;
            std::uint8_t bAlternateSetting;
            std::uint8_t bNumEndpoints;
            std::uint8_t bInterfaceClass;
            std::uint8_t bInterfaceSubClass;
            std::uint8_t bInterfaceProtocol;
            std::uint8_t iInterface{};
        };

        struct InterfaceAssociation
          : detail::DescriptorBase<InterfaceAssociation, DescriptorType::InterfaceAssociation> {
            std::uint8_t bFirstInterface;
            std::uint8_t bInterfaceCount;
            std::uint8_t bFunctionClass;
            std::uint8_t bFunctionSubClass;
            std::uint8_t bFunctionProtocol;
            std::uint8_t iFunction{};
        };

        struct [[gnu::packed]] Endpoint
          : detail::DescriptorBase<Endpoint, DescriptorType::Endpoint> {
            std::uint8_t         bEndpointAddress;
            EndpointTransferType bmAttributes;
            std::uint16_t        wMaxPacketSize;
            std::uint8_t         bInterval{};
        };

        namespace detail {
            template<typename... Ts>
            constexpr auto generateArray(Ts const&... args) {
                std::array<std::byte, (sizeof(Ts) + ...)> buffer;
                auto                                      pos = buffer.begin();
                ((pos = std::copy_n(
                    std::bit_cast<std::array<std::byte, sizeof(args)>>(args).begin(),
                    sizeof(args),
                    pos)),
                 ...);
                return buffer;
            };

        }   // namespace detail

        template<
          std::uint16_t DeviceVersion,
          std::uint16_t VendorID,
          std::uint16_t ProductID,
          std::uint8_t  ManufacturerStringID,
          std::uint8_t  ProductStringID,
          std::uint8_t  SerialNumberStringID,
          DeviceClass   Class,
          DeviceClass   SubClass>
        consteval auto makeDeviceDescriptorArray() {
            constexpr USB::Descriptors::Device DeviceDescriptor{
              .bDeviceClass{Class},
              .bDeviceSubClass{SubClass},
              .bDeviceProtocol{1},
              .bMaxPacketSize0{64},
              .idVendor{VendorID},
              .idProduct{ProductID},
              .bcdDevice{DeviceVersion},
              .iManufacturer{ManufacturerStringID},
              .iProduct{ProductStringID},
              .iSerialNumber{SerialNumberStringID},
              .bNumConfigurations{1}};
            return detail::generateArray(DeviceDescriptor);
        }

        template<typename... Interfaces>
        consteval auto
        makeConfigDescriptorArray(std::uint16_t busPower, Interfaces const&... interfaces) {
            assert(500 >= busPower);
            //TODO
            constexpr USB::Descriptors::InterfaceAssociation InterfaceAssociationDescriptor{
              .bFirstInterface{0},
              .bInterfaceCount{sizeof...(interfaces)},
              .bFunctionClass{2},
              .bFunctionSubClass{2},
              .bFunctionProtocol{0}};

            auto remainingConfig
              = detail::generateArray(InterfaceAssociationDescriptor, interfaces...);

            USB::Descriptors::Configuration ConfigDescriptor{
              .wTotalLength{sizeof(remainingConfig) + sizeof(USB::Descriptors::Configuration)},
              .bNumInterfaces{sizeof...(Interfaces)},
              .bConfigurationValue{1},
              .bMaxPower{static_cast<std::uint8_t>(busPower / 2)}};

            return detail::generateArray(ConfigDescriptor, remainingConfig);
        }

    }   // namespace Descriptors

    namespace SimpleBulk { namespace Descriptors {
            template<
              std::uint16_t DeviceVersion,
              std::uint16_t VendorID,
              std::uint16_t ProductID,
              std::uint8_t  ManufacturerStringID,
              std::uint8_t  ProductStringID,
              std::uint8_t  SerialNumberStringID>
            consteval auto makeDeviceDescriptorArray() {
                return USB::Descriptors::makeDeviceDescriptorArray<
                  DeviceVersion,
                  VendorID,
                  ProductID,
                  ManufacturerStringID,
                  ProductStringID,
                  SerialNumberStringID,
                  DeviceClass::Miscellaneous,
                  DeviceClass::Miscellaneous>();
            }

            template<std::uint8_t DataEndpointID>
            consteval auto makeInterfaceDescriptorArrays() {
                constexpr USB::Descriptors::Interface InterfaceDescriptor{
                  .bInterfaceNumber{0},
                  .bAlternateSetting{0},
                  .bNumEndpoints{2},
                  .bInterfaceClass{255},
                  .bInterfaceSubClass{0},
                  .bInterfaceProtocol{0}};

                constexpr USB::Descriptors::Endpoint DataInEndpointDescriptor{
                  .bEndpointAddress{makeEndpointAddress(EndpointDirection::In, DataEndpointID)},
                  .bmAttributes{EndpointTransferType::Bulk},
                  .wMaxPacketSize{64},
                  .bInterval{0}};

                constexpr USB::Descriptors::Endpoint DataOutEndpointDescriptor{
                  .bEndpointAddress{makeEndpointAddress(EndpointDirection::Out, DataEndpointID)},
                  .bmAttributes{EndpointTransferType::Bulk},
                  .wMaxPacketSize{64},
                  .bInterval{0}};

                constexpr auto Interface = USB::Descriptors::detail::generateArray(
                  InterfaceDescriptor,
                  DataOutEndpointDescriptor,
                  DataInEndpointDescriptor);

                return std::tuple{Interface};
            }
    }}   // namespace SimpleBulk::Descriptors

    namespace CDC {

        struct [[gnu::packed]] LineCoding {
            std::uint32_t dwDTERate{115200};
            std::uint8_t  bCharFormat{};
            std::uint8_t  bParityType{};
            std::uint8_t  bDataBits{8};
        };

        namespace Descriptors {
            struct [[gnu::packed]] Header
              : detail::InterfaceDescriptorBase<Header, DescriptorSubType::CDC_Header> {
                std::uint16_t bcdCDC{0x0120};
            };

            struct Union : detail::InterfaceDescriptorBase<Union, DescriptorSubType::CDC_Union> {
                std::uint8_t bMasterInterface0;
                std::uint8_t bSlaveInterface0;
            };

            struct CallManagement
              : detail::
                  InterfaceDescriptorBase<CallManagement, DescriptorSubType::CDC_CallManagement> {
                std::uint8_t bmCapabilities;
                std::uint8_t bDataInterface;
            };
        }   // namespace Descriptors
        namespace ACM { namespace Descriptors {
                struct Descriptor
                  : detail::
                      InterfaceDescriptorBase<Descriptor, DescriptorSubType::CDC_ACM_Descriptor> {
                    std::uint8_t bmCapabilities;
                };

                template<
                  std::uint16_t DeviceVersion,
                  std::uint16_t VendorID,
                  std::uint16_t ProductID,
                  std::uint8_t  ManufacturerStringID,
                  std::uint8_t  ProductStringID,
                  std::uint8_t  SerialNumberStringID>
                consteval auto makeDeviceDescriptorArray() {
                    return USB::Descriptors::makeDeviceDescriptorArray<
                      DeviceVersion,
                      VendorID,
                      ProductID,
                      ManufacturerStringID,
                      ProductStringID,
                      SerialNumberStringID,
                      DeviceClass::Miscellaneous,
                      DeviceClass::Communication>();
                }

                template<std::uint8_t ManagementEnpointId, std::uint8_t DataEndpointID>
                consteval auto makeInterfaceDescriptorArrays() {
                    constexpr USB::Descriptors::Interface CDC_ManagementInterfaceDescriptor{
                      .bInterfaceNumber{0},
                      .bAlternateSetting{0},
                      .bNumEndpoints{1},
                      .bInterfaceClass{2},
                      .bInterfaceSubClass{2},
                      .bInterfaceProtocol{0}};

                    constexpr USB::Descriptors::Interface ACM_InterfaceDescriptor = {
                      .bInterfaceNumber{1},
                      .bAlternateSetting{0},
                      .bNumEndpoints{2},
                      .bInterfaceClass{10},
                      .bInterfaceSubClass{0},
                      .bInterfaceProtocol{0}};

                    constexpr USB::CDC::Descriptors::Header CDC_HeaderDescriptor{};

                    constexpr USB::CDC ::ACM::Descriptors::Descriptor CDC_ACM_Descriptor{
                      .bmCapabilities{0x00}};

                    constexpr USB::CDC::Descriptors::Union CDC_UnionDescriptor{
                      .bMasterInterface0{0},
                      .bSlaveInterface0{1}};

                    constexpr USB::CDC::Descriptors::CallManagement CDC_CallManagementDescriptor{
                      .bmCapabilities{0x0},
                      .bDataInterface{1}};

                    constexpr USB::Descriptors::Endpoint CDC_ManagementEndpointDescriptor{
                      .bEndpointAddress{
                        makeEndpointAddress(EndpointDirection::In, ManagementEnpointId)},
                      .bmAttributes{EndpointTransferType::Interrupt},
                      .wMaxPacketSize{8},
                      .bInterval{16}};

                    constexpr USB::Descriptors::Endpoint ACM_DataInEndpointDescriptor{
                      .bEndpointAddress{makeEndpointAddress(EndpointDirection::In, DataEndpointID)},
                      .bmAttributes{EndpointTransferType::Bulk},
                      .wMaxPacketSize{64},
                      .bInterval{0}};

                    constexpr USB::Descriptors::Endpoint ACM_DataOutEndpointDescriptor{
                      .bEndpointAddress{
                        makeEndpointAddress(EndpointDirection::Out, DataEndpointID)},
                      .bmAttributes{EndpointTransferType::Bulk},
                      .wMaxPacketSize{64},
                      .bInterval{0}};

                    constexpr auto CDC_Interface = USB::Descriptors::detail::generateArray(
                      CDC_ManagementInterfaceDescriptor,
                      CDC_HeaderDescriptor,
                      CDC_CallManagementDescriptor,
                      CDC_ACM_Descriptor,
                      CDC_UnionDescriptor,
                      CDC_ManagementEndpointDescriptor);
                    constexpr auto ACM_Interface = USB::Descriptors::detail::generateArray(
                      ACM_InterfaceDescriptor,
                      ACM_DataOutEndpointDescriptor,
                      ACM_DataInEndpointDescriptor);
                    return std::tuple{CDC_Interface, ACM_Interface};
                }
        }}   // namespace ACM::Descriptors
    }        // namespace CDC

    namespace RESET { namespace Descriptor {

    }}

    template<typename Buffer>
    struct DescriptorString {
        Buffer buffer{};

        constexpr DescriptorString(std::string_view s) {
            assert(buffer.max_size() >= s.size() * 2);

            if constexpr(requires { buffer.resize(0); }) {
                buffer.resize(s.size() * 2);
            } else {
                assert(buffer.max_size() == s.size() * 2);
            }

            assert(buffer.size() == s.size() * 2);
            auto pos = buffer.begin();
            for(auto c : s) {
                *pos = std::byte{c};
                ++pos;
                *pos = std::byte{0};
                ++pos;
            }
        }

        constexpr std::size_t             size() const { return buffer.size(); }
        constexpr std::byte const*        begin() const { return buffer.begin(); }
        constexpr std::byte const*        end() const { return buffer.end(); }
        constexpr std::byte const*        data() const { return buffer.data(); }
        constexpr DescriptorString const& get() const { return *this; }
    };

    template<std::size_t N>
    DescriptorString(char const (&)[N]) -> DescriptorString<std::array<std::byte, (N - 1) * 2>>;
    DescriptorString(std::string_view s)
      -> DescriptorString<Kvasir::StaticVector<std::byte, 64 * 2>>;

    template<typename F>
    struct RuntimeDescriptorString {
        F f{};
        constexpr RuntimeDescriptorString(F f_) : f{f_} {}

        using DT = DescriptorString<Kvasir::StaticVector<std::byte, 64>>;
        struct RuntimeDescriptorStringImpl : DT {
            template<typename T>
            RuntimeDescriptorStringImpl(T const& v)
              : DT{[&]() {
                  if constexpr(requires { std::string_view{v}; }) {
                      return DT{std::string_view{v}};
                  } else {
                      std::array<char, 64> b;
                      auto const           ret = std::to_chars(b.begin(), b.end(), v);
                      assert(ret.ec == std::errc{});
                      return DT{
                        std::string_view{b.begin(), ret.ptr}
                      };
                  }
              }()} {}
        };

        RuntimeDescriptorStringImpl const get() const { return RuntimeDescriptorStringImpl{f()}; }
    };

    template<typename F>
    RuntimeDescriptorString(F f) -> RuntimeDescriptorString<F>;

    static constexpr DescriptorString winUsbStringDescriptor{"MSFT100"};

    template<typename I, typename T>
    I insertStringDescriptor(T const& descriptor, I first, I last) {
        assert(static_cast<std::size_t>(std::distance(first, last)) >= descriptor.size());
        std::memcpy(first, descriptor.data(), descriptor.size());
        return first + descriptor.size();
    }

    template<typename IT, typename T>
    IT insertStringDescriptor(std::size_t index, T const& descriptors, IT first, IT last) {
        static constexpr auto Size = std::tuple_size_v<T>;
        UC_LOG_D("{:#x}", index);
        if(index == 0xed) {
            first  = insertStringDescriptor(winUsbStringDescriptor.get(), first, last);
            *first = 0x20_b;
            ++first;
            *first = 0x00_b;
            ++first;
            return first;
        }
        assert(Size > index);

        auto action = [&](auto i) {
            static constexpr std::size_t I = decltype(i)::value;
            if(I == index) {
                first = insertStringDescriptor(std::get<I>(descriptors).get(), first, last);
                return true;
            }
            return Size > I;
        };

        bool const ok = [&]<std::size_t... Ns>(std::index_sequence<Ns...>) {
            return (action(std::integral_constant<std::size_t, Ns>{}) && ...);
        }(std::make_index_sequence<Size>{});
        assert(ok);

        return first;
    }

    //
    namespace detail {
        template<typename Clock, typename Child, typename Config>
        struct USBBase {
            static constexpr auto InterruptIndexs
              = brigand::list<decltype(Kvasir::Interrupt::usbctrl)>{};

            using Regs       = Kvasir::Peripheral::USBCTRL_REGS::Registers<0>;
            using BufferRegs = Kvasir::Peripheral::USBCTRL_DPRAM::Registers<0>;

            static inline Kvasir::StaticFunction<void(std::uint16_t), 128> sofFunction{};

            static inline std::atomic<std::uint8_t> deviceBusAddr{};
            static inline std::atomic<bool>         configured{false};
            static inline std::atomic<bool>         shouldSetAddress{false};
            static inline std::atomic<bool>         secondConfig{false};

            static inline std::atomic<bool> next_pid{false};
            static inline std::atomic<bool> next_pid_out{false};

            static void onIsr() {
                auto const status = apply(
                  read(Regs::INTS::setup_req),
                  read(Regs::INTS::buff_status),
                  read(Regs::INTS::bus_reset),
                  read(Regs::INTS::trans_complete),
                  read(Regs::INTS::dev_sof));

                if(get<4>(status)) {
                    auto const sof = get<0>(apply(read(Regs::SOF_RD::count)));
                    if(sofFunction) {
                        sofFunction(static_cast<std::uint16_t>(sof));
                    }
                }

                if(get<0>(status)) {
                    apply(Regs::SIE_STATUS::overrideDefaults(set(Regs::SIE_STATUS::setup_rec)));
                    handleSetupPacket(getSetupPacket());
                }

                if(get<1>(status)) {
                    handleBufferStatus();
                }

                if(get<2>(status)) {
                    apply(Regs::SIE_STATUS::overrideDefaults(set(Regs::SIE_STATUS::bus_reset)));
                    handleBusReset();
                }

                if(get<3>(status)) {
                    apply(
                      Regs::SIE_STATUS::overrideDefaults(set(Regs::SIE_STATUS::trans_complete)));
                }
            }

            template<typename... Ts>
            static constexpr auto makeIsr(brigand::list<Ts...>) {
                return brigand::list<
                  Kvasir::Nvic::Isr<std::addressof(onIsr), Kvasir::Nvic::Index<Ts::value>>...>{};
            }
            using Isr = decltype(makeIsr(InterruptIndexs));

            static constexpr auto powerClockEnable
              = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::usbctrl));

            static constexpr auto initStepPeripheryConfig = list(
              Regs::USB_MUXING::overrideDefaults(set(Regs::USB_MUXING::to_phy)),
              Regs::USB_PWR::overrideDefaults(
                set(Regs::USB_PWR::vbus_detect),
                set(Regs::USB_PWR::vbus_detect_override_en)),
              Regs::MAIN_CTRL::overrideDefaults(set(Regs::MAIN_CTRL::controller_en)),
              Regs::INTE::overrideDefaults(
                set(Regs::INTE::buff_status),
                set(Regs::INTE::bus_reset),
                set(Regs::INTE::setup_req),
                set(Regs::INTE::trans_complete),
                set(Regs::INTE::dev_sof)));

            static constexpr auto initStepInterruptConfig = list(
              Kvasir::Nvic::makeSetPriority<0>(InterruptIndexs),
              Kvasir::Nvic::makeClearPending(InterruptIndexs));

            static void endpointConfig() { Child::setupEndpoints(); }

            static constexpr auto runtimeInit = []() {
                std::memset(reinterpret_cast<void*>(BufferRegs::baseAddr), 0, 4092);

                endpointConfig();
                apply(Kvasir::Nvic::makeEnable(InterruptIndexs));

                apply(Regs::SIE_CTRL::overrideDefaults(
                  set(Regs::SIE_CTRL::pullup_en),
                  set(Regs::SIE_CTRL::ep0_int_1buf)));
            };

            static Kvasir::USB::SetupPacket getSetupPacket() {
                Kvasir::USB::SetupPacket ret;
                std::memcpy(
                  std::addressof(ret),
                  reinterpret_cast<void const*>(BufferRegs::SETUP_PACKET_LOW::Addr::value),
                  sizeof(Kvasir::USB::SetupPacket));
                return ret;
            }

            static void handleBusReset() {
                UC_LOG_D("reset");
                deviceBusAddr    = 0;
                shouldSetAddress = false;
                configured       = false;

                apply(write(Regs::EP<0>::ADDR_ENDP::address, Kvasir::Register::value<0>()));

                Child::handleReset();
            }

            static void ep0Handler(bool in) {
                if(in) {
                    if(shouldSetAddress) {
                        // Set actual device address in hardware
                        UC_LOG_D("new address {}", deviceBusAddr.load());
                        apply(write(Regs::EP<0>::ADDR_ENDP::address, deviceBusAddr.load()));

                        shouldSetAddress = false;
                    } else {
                        if constexpr(Child::ConfigDescriptor.size() > 64) {
                            if(secondConfig) {
                                secondConfig = false;
                                ep0IN(std::span{
                                  Child::ConfigDescriptor.begin() + 64,
                                  Child::ConfigDescriptor.end()});
                            } else {
                                Clock::template delay<std::chrono::microseconds, 50>();
                                ep0OUT(0);
                            }
                        } else {
                            Clock::template delay<std::chrono::microseconds, 50>();
                            ep0OUT(0);
                        }
                    }
                }
            }

            static void handleBufferDone(std::size_t ep_num, bool in) {
                if(ep_num == 0) {
                    ep0Handler(in);
                } else {
                    Child::epHandler(ep_num, in);
                }
            }

            static void handleBufferStatus() {
                std::uint32_t buffers = *reinterpret_cast<std::uint32_t volatile*>(
                  Kvasir::Peripheral::USBCTRL_REGS::Registers<0>::BUFF_STATUS::Addr::value);
                std::uint32_t remaining_buffers = buffers;
                std::size_t   bit               = 1u;
                for(std::size_t i = 0; remaining_buffers && i < 16 * 2; i++) {
                    if(remaining_buffers & bit) {
                        // clear this in advance
                        *reinterpret_cast<std::uint32_t volatile*>(
                          Kvasir::Peripheral::USBCTRL_REGS::Registers<0>::BUFF_STATUS::Addr::value)
                          = bit;
                        // IN transfer for even i, OUT transfer for odd i
                        handleBufferDone(i >> 1u, !(i & 1u));
                        remaining_buffers &= ~bit;
                    }
                    bit <<= 1u;
                }
            }

            template<std::size_t EP, std::size_t Buffer>
            static void startINTransfer(std::span<std::byte const> data, bool pid) {
                static_assert(Buffer == 1 || Buffer == 0, "only dual buffered");
                assert(64 >= data.size());
                using B = BufferRegs::DOUBLEBUFFER<EP>;

                if(!data.empty()) {
                    if constexpr(Buffer == 0) {
                        std::memcpy(
                          reinterpret_cast<void*>(B::template IN<0>::Addr::value),
                          data.data(),
                          data.size());
                    } else {
                        std::memcpy(
                          reinterpret_cast<void*>(B::template IN<1>::Addr::value),
                          data.data(),
                          data.size());
                    }
                }

                if constexpr(Buffer == 0) {
                    using BC = typename BufferRegs::EP<EP>::IN_BUFFER_CONTROL_0;
                    apply(
                      clear(BC::last),
                      set(BC::full),
                      write(BC::pid, pid ? 1 : 0),
                      write(BC::length, static_cast<std::uint16_t>(data.size())),
                      set(BC::available));
                } else {
                    using BC = typename BufferRegs::EP<EP>::IN_BUFFER_CONTROL_1;
                    apply(
                      clear(BC::last),
                      set(BC::full),
                      write(BC::pid, pid ? 1 : 0),
                      write(BC::length, static_cast<std::uint16_t>(data.size())),
                      set(BC::available));
                }
            }

            template<std::size_t EP, std::size_t Buffer>
            static void startOUTTransfer(std::size_t size, bool pid) {
                static_assert(Buffer == 1 || Buffer == 0, "only dual buffered");
                assert(64 >= size);

                if constexpr(Buffer == 0) {
                    using BC = typename BufferRegs::EP<EP>::OUT_BUFFER_CONTROL_0;
                    apply(
                      clear(BC::last),
                      clear(BC::full),
                      write(BC::pid, pid ? 1 : 0),
                      write(BC::length, static_cast<std::uint16_t>(size)),
                      set(BC::available));
                } else {
                    using BC = typename BufferRegs::EP<EP>::OUT_BUFFER_CONTROL_1;
                    apply(
                      clear(BC::last),
                      clear(BC::full),
                      write(BC::pid, pid ? 1 : 0),
                      write(BC::length, static_cast<std::uint16_t>(size)),
                      set(BC::available));
                }
            }

            static void ep0IN(std::span<std::byte const> data) {
                startINTransfer<0, 0>(data, next_pid);
                next_pid = !next_pid;
            }

            static void ep0OUT(std::size_t size) {
                startOUTTransfer<0, 0>(size, next_pid_out);
                next_pid_out = !next_pid_out;
            }

            static void acknowledgeSetupRequest() { ep0IN(std::span<std::byte const>{}); }

            static void handleDeviceDescriptor() { ep0IN(Child::DeviceDescriptor); }

            static void handleConfigDescriptor(Kvasir::USB::SetupPacket const& pkt) {
                if(pkt.wLength >= Child::ConfigDescriptor.size()) {
                    if constexpr(Child::ConfigDescriptor.size() > 64) {
                        secondConfig = true;
                        ep0IN(std::span{
                          Child::ConfigDescriptor.begin(),
                          Child::ConfigDescriptor.begin() + 64});
                    } else {
                        ep0IN(Child::ConfigDescriptor);
                    }
                } else {
                    ep0IN(std::span{
                      Child::ConfigDescriptor.begin(),
                      Child::ConfigDescriptor.begin()
                        + sizeof(Kvasir::USB::Descriptors::Configuration)});
                }
            }

            static void handleStringDescriptor(Kvasir::USB::SetupPacket const& pkt) {
                auto const                index = static_cast<std::size_t>(pkt.wValue & 0xff);
                std::uint8_t              len{};
                std::array<std::byte, 64> buffer;
                if(index == 0) {
                    buffer[2] = std::byte{0x09};
                    buffer[3] = std::byte{0x04};
                    len       = 4;
                } else {
                    auto const end = Kvasir::USB::insertStringDescriptor(
                      index - 1,
                      Child::DescriptorStrings,
                      buffer.begin() + 2,
                      buffer.end());
                    len = static_cast<std::uint8_t>(std::distance(buffer.begin(), end));
                }

                buffer[0] = std::byte{len};
                buffer[1] = std::byte(Kvasir::USB::SetupPacket::DescriptorType::string);
                UC_LOG_D(
                  "request string with {} {}",
                  index,
                  std::span{buffer.data(), std::min<std::uint16_t>(pkt.wLength, len)});
                ep0IN(std::span{buffer.data(), std::min<std::uint16_t>(pkt.wLength, len)});
            }

            static void handleSetupPacketDeviceIn(Kvasir::USB::SetupPacket const& pkt) {
                if(pkt.bRequest == Kvasir::USB::SetupPacket::Request::getDescriptor) {
                    switch(pkt.descriptorType()) {
                    case Kvasir::USB::SetupPacket::DescriptorType::device:
                        {
                            handleDeviceDescriptor();
                        }
                        break;
                    case Kvasir::USB::SetupPacket::DescriptorType::configuration:
                        {
                            handleConfigDescriptor(pkt);
                        }
                        break;
                    case Kvasir::USB::SetupPacket::DescriptorType::string:
                        {
                            handleStringDescriptor(pkt);
                        }
                        break;
                    case Kvasir::USB::SetupPacket::DescriptorType::deviceQualifier:
                        {
                            acknowledgeSetupRequest();
                        }
                        break;
                    default:
                        {
                            UC_LOG_W("Other getDescriptor");
                        }
                        break;
                    }
                } else if(pkt.bRequest == Kvasir::USB::SetupPacket::Request::getStatus) {
                    UC_LOG_W("status IN request {}", pkt.bRequest);
                    std::array<std::byte, 2> buffer{};

                    ep0IN(std::span{buffer.data(), buffer.size()});
                } else {
                    if(
                      pkt.type() == Kvasir::USB::SetupPacket::Type::vendor
                      && pkt.bRequest == Kvasir::USB::SetupPacket::Request::setLineCoding)
                    {
                        std::array<std::byte, 40> foo{
                          0x28_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x01_b, 0x04_b, 0x00_b,
                          0x01_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b,
                          0x00_b, 0x01_b, 0x57_b, 0x49_b, 0x4E_b, 0x55_b, 0x53_b, 0x42_b,
                          0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b,
                          0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b, 0x00_b};

                        ep0IN(
                          std::span{foo.data(), std::min<std::uint16_t>(pkt.wLength, foo.size())});
                    } else {
                        UC_LOG_W("Other IN request {}", pkt.bRequest);
                    }
                }
            }
            static void handleSetupPacketDeviceOut(Kvasir::USB::SetupPacket const& pkt) {
                switch(pkt.bRequest) {
                case Kvasir::USB::SetupPacket::Request::setAddress:
                    {
                        // Set address is a bit of a strange case because we have to send a 0 length status packet first with address 0
                        deviceBusAddr = pkt.wValue & 0xff;
                        // Will set address in the callback phase
                        shouldSetAddress = true;
                        acknowledgeSetupRequest();
                    }
                    break;
                case Kvasir::USB::SetupPacket::Request::setConfiguration:
                    {
                        acknowledgeSetupRequest();
                        configured = true;
                        Child::ConfiguredCallback();
                    }
                    break;
                default:
                    {
                        UC_LOG_W("ack missing out");
                        acknowledgeSetupRequest();
                    }
                    break;
                }
            }

            static void handleSetupPacket(Kvasir::USB::SetupPacket const& pkt) {
                UC_LOG_D(
                  "setup packet {} {} {} {} {} {} {} {}",
                  pkt.direction(),
                  pkt.type(),
                  pkt.recipiant(),
                  pkt.descriptorType(),
                  pkt.bRequest,
                  pkt.wIndex,
                  pkt.wValue,
                  pkt.wLength);

                next_pid = true;

                switch(pkt.recipiant()) {
                case Kvasir::USB::SetupPacket::Recipiant::device:
                    {
                        if(pkt.direction() == Kvasir::USB::SetupPacket::Direction::hostToDevice) {
                            handleSetupPacketDeviceOut(pkt);
                        } else {
                            handleSetupPacketDeviceIn(pkt);
                        }
                    }
                    break;
                case Kvasir::USB::SetupPacket::Recipiant::interface:
                    {
                        Child::SetupPacketInterface(pkt);
                    }
                    break;
                case Kvasir::USB::SetupPacket::Recipiant::endpoint:
                    {
                        Child::SetupPacketEndpoint(pkt);
                    }
                    break;

                case Kvasir::USB::SetupPacket::Recipiant::other:
                    {
                        UC_LOG_W("foo");
                    }
                    break;
                default:
                    {
                        UC_LOG_W("foo");
                    }
                    break;
                }
            }
        };
    }   // namespace detail

    template<typename Clock, typename Config>
    struct USB_CDC_ACM : detail::USBBase<Clock, USB_CDC_ACM<Clock, Config>, Config> {
        using Base       = detail::USBBase<Clock, USB_CDC_ACM<Clock, Config>, Config>;
        using Regs       = typename Base::Regs;
        using BufferRegs = typename Base::BufferRegs;

        static constexpr std::tuple DescriptorStrings{
          Kvasir::USB::DescriptorString{Config::ManufacturerString},
          Kvasir::USB::DescriptorString{Config::ProductString},
          Kvasir::USB::RuntimeDescriptorString{Config::SerialNumberString}};

        static constexpr auto DeviceDescriptor{
          Kvasir::USB::CDC::ACM::Descriptors::makeDeviceDescriptorArray<
            Config::ProductVersionBCD,
            Config::VendorID,
            Config::ProductID,
            1,
            2,
            3>()};

        static constexpr auto InterfaceDescriptors{
          Kvasir::USB::CDC::ACM::Descriptors::makeInterfaceDescriptorArrays<1, 2>()};

        static constexpr auto ConfigDescriptor{Kvasir::USB::Descriptors::makeConfigDescriptorArray(
          500,
          std::get<0>(InterfaceDescriptors),
          std::get<1>(InterfaceDescriptors),
          Kvasir::USB::Descriptors::Interface{
            .bInterfaceNumber   = 2,
            .bAlternateSetting  = 0,
            .bNumEndpoints      = 0,
            .bInterfaceClass    = 255,
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 1,
            .iInterface         = 0}

          )};

        static void setupEndpoints() {
            {
                using EP     = typename Base::BufferRegs::template EP<1>::IN_CONTROL;
                using Buffer = typename Base::BufferRegs::template DOUBLEBUFFER<1>::template IN<0>;
                apply(EP::overrideDefaults(
                  set(EP::enable),
                  write(EP::ENDPOINT_TYPEValC::interrupt),
                  set(EP::double_buffered),
                  set(EP::interrupt_per_buff),
                  write(
                    EP::buffer_address,
                    Kvasir::Register::value<Buffer::Addr::value - Base::BufferRegs::baseAddr>())));
            }
            {
                using EP     = typename Base::BufferRegs::template EP<2>::IN_CONTROL;
                using Buffer = typename Base::BufferRegs::template DOUBLEBUFFER<2>::template IN<0>;
                apply(EP::overrideDefaults(
                  set(EP::enable),
                  write(EP::ENDPOINT_TYPEValC::bulk),
                  set(EP::double_buffered),
                  set(EP::interrupt_per_buff),
                  write(
                    EP::buffer_address,
                    Kvasir::Register::value<Buffer::Addr::value - Base::BufferRegs::baseAddr>())));
            }
            {
                using EP     = typename Base::BufferRegs::template EP<2>::OUT_CONTROL;
                using Buffer = typename Base::BufferRegs::template DOUBLEBUFFER<2>::template OUT<0>;
                apply(EP::overrideDefaults(
                  set(EP::enable),
                  write(EP::ENDPOINT_TYPEValC::bulk),
                  set(EP::double_buffered),
                  set(EP::interrupt_per_buff),
                  write(
                    EP::buffer_address,
                    Kvasir::Register::value<Buffer::Addr::value - Base::BufferRegs::baseAddr>())));
            }
        }

        static void epHandler(std::size_t epNum, bool in) {
            if(epNum == 2 && in) {
                sendNext();
            } else if(epNum == 2 && !in) {
                ep2OUTHandler();
            }
        }

        static inline std::atomic<bool>                      acm_connected = false;
        static inline std::span<std::byte const>             currentSendData{};
        static inline bool                                   next_pid3   = false;
        static inline bool                                   buffer_cnt3 = false;
        static inline bool                                   next_pid2   = false;
        static inline bool                                   buffer_cnt  = false;
        static inline std::atomic<bool>                      sendRdy     = true;
        static inline Kvasir::Atomic::Queue<std::byte, 4096> recvBuffer{};

        static void handleReset() {   //todo reset stuff
            acm_connected   = false;
            currentSendData = std::span<std::byte const>{};
            next_pid3       = false;
            buffer_cnt3     = false;
            next_pid2       = false;
            buffer_cnt      = false;
            sendRdy         = true;
            recvBuffer.clear();
        }

        static void ConfiguredCallback() { recv(); }
        static void ep2OUTHandler() {
            bool const buffer = apply(read(Regs::BUFF_CPU_SHOULD_HANDLE::ep2_out));

            std::array<std::byte, 64> x{};
            std::size_t               len{};
            if(!buffer) {
                len     = apply(read(BufferRegs::template EP<2>::OUT_BUFFER_CONTROL_0::length));
                using B = typename BufferRegs::template DOUBLEBUFFER<2>;
                std::memcpy(
                  x.data(),
                  reinterpret_cast<void*>(B::template OUT<0>::Addr::value),
                  len);
            } else {
                len     = apply(read(BufferRegs::template EP<2>::OUT_BUFFER_CONTROL_1::length));
                using B = typename BufferRegs::template DOUBLEBUFFER<2>;
                std::memcpy(x.data(), reinterpret_cast<void*>(B::template OUT<1>::Addr::value), 64);
            }
            recvBuffer.push(std::span{x.data(), len});
            recv();
        }

        static bool recv() {
            static constexpr std::size_t EP = 2;
            using Buffer = typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL;

            auto const av = apply(read(Buffer::available_0), read(Buffer::available_1));
            if(get<0>(av) && get<1>(av)) {
                return false;
            }

            if(!get<0>(av) && !buffer_cnt3) {
                using BC = typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL_0;
                apply(
                  clear(BC::last),
                  clear(BC::full),
                  write(BC::pid, next_pid3 ? 1 : 0),
                  write(BC::length, Kvasir::Register::value<std::uint16_t, 64>()),
                  set(BC::available));
            } else if(!get<1>(av) && buffer_cnt3) {
                using BC = typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL_1;

                apply(
                  clear(BC::last),
                  clear(BC::full),
                  write(BC::pid, next_pid3 ? 1 : 0),
                  write(BC::length, Kvasir::Register::value<std::uint16_t, 64>()),
                  set(BC::available));
            } else {
                return false;
            }
            buffer_cnt3 = !buffer_cnt3;
            next_pid3   = !next_pid3;
            return false;
        }

        static inline USB::CDC::LineCoding lineConding{};

        static void SetupPacketInterface(Kvasir::USB::SetupPacket const& pkt) {
            if(
              pkt.wIndex == 0
              && pkt.direction() == Kvasir::USB::SetupPacket::Direction::hostToDevice
              && pkt.bRequest == Kvasir::USB::SetupPacket::Request::setControlLineState)
            {
                if((pkt.wValue & 0x01) == 0) {
                    UC_LOG_D("disconnected");
                    acm_connected = false;
                } else {
                    UC_LOG_D("connected");
                    acm_connected = true;
                }
                Base::acknowledgeSetupRequest();
            } else if(
              pkt.wIndex == 0
              && pkt.direction() == Kvasir::USB::SetupPacket::Direction::deviceToHost
              && (pkt.bRequest == Kvasir::USB::SetupPacket::Request::getLineCoding))
            {
                recv();
                UC_LOG_D("get line coding");
                Base::ep0IN(std::as_bytes(std::span{std::addressof(lineConding), 1}));
            } else if(
              pkt.wIndex == 0
              && pkt.direction() == Kvasir::USB::SetupPacket::Direction::hostToDevice
              && pkt.bRequest == Kvasir::USB::SetupPacket::Request::setLineCoding)
            {
                UC_LOG_D("set line coding");
                using B = typename BufferRegs::template BUFFER<0>::IN;
                std::array<std::byte, 7> data;
                std::memcpy(data.data(), reinterpret_cast<void*>(B::Addr::value), 7);
                data[0] = std::byte{0};

                UC_LOG_D("{}", data);
                std::memcpy(std::addressof(lineConding), data.data(), 7);

                Base::acknowledgeSetupRequest();
            } else if(pkt.wIndex == 2) {
                static constexpr Kvasir::USB::SetupPacket::Request REQUEST_BOOTSEL{0x01};

                static constexpr Kvasir::USB::SetupPacket::Request REQUEST_FLASH{0x02};

                if(pkt.bRequest == REQUEST_BOOTSEL) {
                    UC_LOG_D("reboot to bootsel");
                    Kvasir::resetToUsbBoot();
                } else if(pkt.bRequest == REQUEST_FLASH) {
                    UC_LOG_D("reboot");
                    apply(Kvasir::SystemControl::SystemReset{});
                } else {
                    UC_LOG_D("unknown message on reset interface");
                }
            } else {
                Base::acknowledgeSetupRequest();
                UC_LOG_D(
                  "bad interface {} {} {} {} {}",
                  pkt.direction(),
                  pkt.bRequest,
                  pkt.wIndex,
                  pkt.wValue,
                  pkt.wLength);
            }
        }
        static void SetupPacketEndpoint(Kvasir::USB::SetupPacket const& pkt) {
            UC_LOG_D("setup endpoint {}", pkt.wIndex);
            if(
              pkt.direction() == Kvasir::USB::SetupPacket::Direction::hostToDevice
              && pkt.bRequest == Kvasir::USB::SetupPacket::Request::clearFeature)
            {
                Base::acknowledgeSetupRequest();
            } else {
                UC_LOG_D("foo");
            }
        }

        static bool send(std::span<std::byte const> data) {
            using Buffer = typename Base::BufferRegs::template EP<2>::IN_BUFFER_CONTROL;

            assert(64 >= data.size());
            auto const av = apply(read(Buffer::available_0), read(Buffer::available_1));
            if(get<0>(av) && get<1>(av)) {
                return false;
            }

            if(!get<0>(av) && !buffer_cnt) {
                Base::template startINTransfer<2, 0>(data, next_pid2);
            } else if(!get<1>(av) && buffer_cnt) {
                Base::template startINTransfer<2, 1>(data, next_pid2);
            } else {
                UC_LOG_D("foobar");
                return false;
            }

            buffer_cnt = !buffer_cnt;
            next_pid2  = !next_pid2;
            return true;
        }

        static void sendNext() {
            if(currentSendData.size() > 64) {
                auto const sub  = currentSendData.subspan(0, 64);
                currentSendData = currentSendData.subspan(64);
                send(sub);
            } else if(!currentSendData.empty()) {
                auto const sub  = currentSendData;
                currentSendData = std::span<std::byte const>{};
                send(sub);
            } else {
                sendRdy = true;
            }
        }
        static void send_nocopy(std::span<std::byte const> data) {
            assert(sendRdy == true);
            currentSendData = data;
            sendRdy         = false;
            sendNext();
        }
    };

    template<typename Clock, typename Config>
    struct USB_Bulk : detail::USBBase<Clock, USB_Bulk<Clock, Config>, Config> {
        using Base       = detail::USBBase<Clock, USB_Bulk<Clock, Config>, Config>;
        using Regs       = typename Base::Regs;
        using BufferRegs = typename Base::BufferRegs;

        static constexpr std::tuple DescriptorStrings{
          Kvasir::USB::DescriptorString{Config::ManufacturerString},
          Kvasir::USB::DescriptorString{Config::ProductString},
          Kvasir::USB::RuntimeDescriptorString{Config::SerialNumberString}};

        static constexpr auto DeviceDescriptor{
          Kvasir::USB::SimpleBulk::Descriptors::makeDeviceDescriptorArray<
            Config::ProductVersionBCD,
            Config::VendorID,
            Config::ProductID,
            1,
            2,
            3>()};

        static constexpr auto InterfaceDescriptors{
          Kvasir::USB::SimpleBulk::Descriptors::makeInterfaceDescriptorArrays<1>()};

        static constexpr auto ConfigDescriptor{Kvasir::USB::Descriptors::makeConfigDescriptorArray(
          500,
          std::get<0>(InterfaceDescriptors),
          Kvasir::USB::Descriptors::Interface{
            .bInterfaceNumber   = 1,
            .bAlternateSetting  = 0,
            .bNumEndpoints      = 0,
            .bInterfaceClass    = 255,
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 1,
            .iInterface         = 0})};

        static void setupEndpoints() {
            {
                using EP     = typename Base::BufferRegs::template EP<1>::IN_CONTROL;
                using Buffer = typename Base::BufferRegs::template DOUBLEBUFFER<1>::template IN<0>;
                apply(EP::overrideDefaults(
                  set(EP::enable),
                  write(EP::ENDPOINT_TYPEValC::bulk),
                  set(EP::double_buffered),
                  set(EP::interrupt_per_buff),
                  write(
                    EP::buffer_address,
                    Kvasir::Register::value<Buffer::Addr::value - Base::BufferRegs::baseAddr>())));
            }
            {
                using EP     = typename Base::BufferRegs::template EP<1>::OUT_CONTROL;
                using Buffer = typename Base::BufferRegs::template DOUBLEBUFFER<1>::template OUT<0>;
                apply(EP::overrideDefaults(
                  set(EP::enable),
                  write(EP::ENDPOINT_TYPEValC::bulk),
                  set(EP::double_buffered),
                  set(EP::interrupt_per_buff),
                  write(
                    EP::buffer_address,
                    Kvasir::Register::value<Buffer::Addr::value - Base::BufferRegs::baseAddr>())));
            }
        }

        static void epHandler(std::size_t epNum, bool in) {
            if(epNum == 1 && in) {
                ep1INHandler();
            } else if(epNum == 1 && !in) {
                ep1OUTHandler();
            }
        }

        static inline std::span<std::byte const>             currentSendData{};
        static inline bool                                   next_pid3   = false;
        static inline bool                                   buffer_cnt3 = false;
        static inline bool                                   next_pid2   = false;
        static inline bool                                   buffer_cnt  = false;
        static inline std::atomic<bool>                      sendRdy     = true;
        static inline Kvasir::Atomic::Queue<std::byte, 4096> recvBuffer{};

        static void handleReset() {   //todo reset stuff
            currentSendData = std::span<std::byte const>{};
            next_pid3       = false;
            buffer_cnt3     = false;
            next_pid2       = false;
            buffer_cnt      = false;
            sendRdy         = true;
            recvBuffer.clear();
        }
        static void ConfiguredCallback() { recv(); }

        static void ep1INHandler() { sendNext(); }

        static void ep1OUTHandler() {
            bool const buffer = apply(read(Regs::BUFF_CPU_SHOULD_HANDLE::ep1_out));

            std::array<std::byte, 64> x{};
            std::size_t               len{};
            if(!buffer) {
                len     = apply(read(BufferRegs::template EP<1>::OUT_BUFFER_CONTROL_0::length));
                using B = typename BufferRegs::template DOUBLEBUFFER<1>;
                std::memcpy(
                  x.data(),
                  reinterpret_cast<void*>(B::template OUT<0>::Addr::value),
                  len);
            } else {
                len     = apply(read(BufferRegs::template EP<1>::OUT_BUFFER_CONTROL_1::length));
                using B = typename BufferRegs::template DOUBLEBUFFER<1>;
                std::memcpy(x.data(), reinterpret_cast<void*>(B::template OUT<1>::Addr::value), 64);
            }
            recvBuffer.push(std::span{x.data(), len});
            recv();
        }

        static bool recv() {
            static constexpr std::size_t EP = 1;
            using Buffer = typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL;

            auto const av = apply(read(Buffer::available_0), read(Buffer::available_1));
            if(get<0>(av) && get<1>(av)) {
                return false;
            }

            if(!get<0>(av) && !buffer_cnt3) {
                using BC = typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL_0;
                apply(
                  clear(BC::last),
                  clear(BC::full),
                  write(BC::pid, next_pid3 ? 1 : 0),
                  write(BC::length, Kvasir::Register::value<std::uint16_t, 64>()),
                  set(BC::available));
            } else if(!get<1>(av) && buffer_cnt3) {
                using BC = typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL_1;

                apply(
                  clear(BC::last),
                  clear(BC::full),
                  write(BC::pid, next_pid3 ? 1 : 0),
                  write(BC::length, Kvasir::Register::value<std::uint16_t, 64>()),
                  set(BC::available));
            } else {
                return false;
            }
            buffer_cnt3 = !buffer_cnt3;
            next_pid3   = !next_pid3;
            return false;
        }

        static void SetupPacketInterface([[maybe_unused]] Kvasir::USB::SetupPacket const& pkt) {
            Base::acknowledgeSetupRequest();

            if(pkt.wIndex == 1) {
                static constexpr Kvasir::USB::SetupPacket::Request REQUEST_BOOTSEL{0x01};

                static constexpr Kvasir::USB::SetupPacket::Request REQUEST_FLASH{0x02};

                if(pkt.bRequest == REQUEST_BOOTSEL) {
                    UC_LOG_D("reboot to bootsel");
                    Kvasir::resetToUsbBoot();
                } else if(pkt.bRequest == REQUEST_FLASH) {
                    UC_LOG_D("reboot");
                    apply(Kvasir::SystemControl::SystemReset{});
                } else {
                    UC_LOG_D("unknown message on reset interface");
                }
            } else {
                UC_LOG_D(
                  "bad interface {} {} {} {} {}",
                  pkt.direction(),
                  pkt.bRequest,
                  pkt.wIndex,
                  pkt.wValue,
                  pkt.wLength);
            }
        }

        static void SetupPacketEndpoint([[maybe_unused]] Kvasir::USB::SetupPacket const& pkt) {
            UC_LOG_D("setup endpoint {}", pkt.wIndex);

            Base::acknowledgeSetupRequest();
            UC_LOG_D(
              "bad endpoint {} {} {} {} {}",
              pkt.direction(),
              pkt.bRequest,
              pkt.wIndex,
              pkt.wValue,
              pkt.wLength);
        }

        static bool send(std::span<std::byte const> data) {
            using Buffer = typename Base::BufferRegs::template EP<1>::IN_BUFFER_CONTROL;

            assert(64 >= data.size());
            auto const av = apply(read(Buffer::available_0), read(Buffer::available_1));
            if(get<0>(av) && get<1>(av)) {
                return false;
            }

            if(!get<0>(av) && !buffer_cnt) {
                Base::template startINTransfer<1, 0>(data, next_pid2);
            } else if(!get<1>(av) && buffer_cnt) {
                Base::template startINTransfer<1, 1>(data, next_pid2);
            } else {
                UC_LOG_D("foobar");
                return false;
            }

            buffer_cnt = !buffer_cnt;
            next_pid2  = !next_pid2;
            return true;
        }

        static void sendNext() {
            if(currentSendData.size() > 64) {
                auto const sub  = currentSendData.subspan(0, 64);
                currentSendData = currentSendData.subspan(64);
                send(sub);
            } else if(!currentSendData.empty()) {
                auto const sub  = currentSendData;
                currentSendData = std::span<std::byte const>{};
                send(sub);
            } else {
                sendRdy = true;
            }
        }

        static void send_nocopy(std::span<std::byte const> data) {
            assert(sendRdy == true);
            currentSendData = data;
            sendRdy         = false;
            sendNext();
        }
    };
}}   // namespace Kvasir::USB
