#pragma once

#include "Io.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/I2C.hpp"

#include <array>
#include <cassert>
#include <span>

namespace Kvasir { namespace I2C {

    namespace Detail {

        enum class PinType { sda0, scl0, sda1, scl1 };

        static constexpr std::array<PinType, 30> I2cPins{
          PinType::sda0, PinType::scl0, PinType::sda1, PinType::scl1, PinType::sda0, PinType::scl0,
          PinType::sda1, PinType::scl1, PinType::sda0, PinType::scl0, PinType::sda1, PinType::scl1,
          PinType::sda0, PinType::scl0, PinType::sda1, PinType::scl1, PinType::sda0, PinType::scl0,
          PinType::sda1, PinType::scl1, PinType::sda0, PinType::scl0, PinType::sda1, PinType::scl1,
          PinType::sda0, PinType::scl0, PinType::sda1, PinType::scl1, PinType::sda0, PinType::scl0};

        template<unsigned Instance>
        struct Config {
            using Regs = Kvasir::Peripheral::I2C::Registers<Instance>;

            template<int Port, int Pin>
            static constexpr bool isValidPinLocationSDA(Kvasir::Register::PinLocation<Port, Pin>) {
                return Instance == 0 ? I2cPins[Pin] == PinType::sda0
                                     : I2cPins[Pin] == PinType::sda1;
            }

            template<int Port, int Pin>
            static constexpr bool isValidPinLocationSCL(Kvasir::Register::PinLocation<Port, Pin>) {
                return Instance == 0 ? I2cPins[Pin] == PinType::scl0
                                     : I2cPins[Pin] == PinType::scl1;
            }

            template<typename SDAPIN>
            struct GetSDAPinConfig;

            template<int Port, int Pin>
            struct GetSDAPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<3>{},
                  Register::PinLocation<Port, Pin>{}));
            };
            template<typename SCLPIN>
            struct GetSCLPinConfig;

            template<int Port, int Pin>
            struct GetSCLPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<3>{},
                  Register::PinLocation<Port, Pin>{}));
            };
            /*
            template<Mode mode>
            struct GetModeConfig {
                static constexpr auto config_ = []() {
                    if constexpr(mode == Mode::_0) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<0>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<0>()))>{};
                    } else if constexpr(mode == Mode::_1) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<0>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<1>()))>{};
                    } else if constexpr(mode == Mode::_2) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<1>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<0>()))>{};
                    } else if constexpr(mode == Mode::_3) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<1>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<1>()))>{};
                    }
                }();
                using config = decltype(config_);
            };
*/
            static constexpr double
            calcf_Baud(std::uint32_t f_clockSpeed, std::uint32_t scr, std::uint32_t cpsdvsr) {
                return (double(f_clockSpeed) / (double(cpsdvsr) * (1.0 + double(scr))));
            }

            static constexpr std::pair<std::uint8_t, std::uint8_t>
            calcBaudRegs(std::uint32_t f_clockSpeed, std::uint32_t f_baud) {
                std::pair<std::uint8_t, std::uint8_t> ret{};
                double                                best = std::numeric_limits<double>::max();

                for(std::uint32_t cpsdvsr = 2; cpsdvsr < 255; cpsdvsr += 2) {
                    for(std::uint32_t scr = 0; scr < 256; ++scr) {
                        double f_div     = f_baud - calcf_Baud(f_clockSpeed, scr, cpsdvsr);
                        double abs_f_div = f_div > 0.0 ? f_div : -f_div;
                        if(best > abs_f_div) {
                            best       = abs_f_div;
                            ret.first  = static_cast<std::uint8_t>(scr);
                            ret.second = static_cast<std::uint8_t>(cpsdvsr);
                            if(abs_f_div == 0.0) {
                                return ret;
                            }
                        }
                    }
                }
                return ret;
            }

            template<
              std::uint32_t f_clockSpeed,
              std::uint32_t f_baud,
              std::intmax_t Num,
              std::intmax_t Denom>
            static constexpr bool isValidBaudConfig(std::ratio<Num, Denom>) {
                constexpr auto baudRegs     = calcBaudRegs(f_clockSpeed, f_baud);
                constexpr auto scr          = std::get<0>(baudRegs);
                constexpr auto cpsdvsr      = std::get<1>(baudRegs);
                constexpr auto f_baudCalced = calcf_Baud(f_clockSpeed, scr, cpsdvsr);
                constexpr auto err          = f_baudCalced - double(f_baud);
                constexpr auto absErr       = err > 0.0 ? err : -err;
                constexpr auto ret = absErr <= (double(f_baud) * (double(Num) / (double(Denom))));
                return (cpsdvsr >= 2) && (cpsdvsr % 2 == 0) && ret;
            }

            template<std::uint32_t f_clockSpeed, std::uint32_t f_baud>
            static constexpr auto getBaudConfig() {
                constexpr auto baudRegs = calcBaudRegs(f_clockSpeed, f_baud);
                return list(
                  write(Regs::SSPCR0::scr, Register::value<std::get<0>(baudRegs)>()),
                  write(Regs::SSPCPSR::cpsdvsr, Register::value<std::get<1>(baudRegs)>()));
            }
        };
    }   // namespace Detail

    namespace Traits { namespace I2C {
            template<unsigned Instance>
            static constexpr auto getIsrIndexs() {
                if constexpr(Instance == 0) {
                    return brigand::list<decltype(Kvasir::Interrupt::i2c0)>{};

                } else if(Instance == 1) {
                    return brigand::list<decltype(Kvasir::Interrupt::i2c1)>{};
                }
            }

            template<unsigned Instance>
            static constexpr auto getEnable() {
                if constexpr(Instance == 0) {
                    return clear(Peripheral::RESETS::Registers<>::RESET::i2c0);
                } else if(Instance == 1) {
                    return clear(Peripheral::RESETS::Registers<>::RESET::i2c1);
                }
            }
            template<unsigned Instance>
            static constexpr auto getDisable() {
                if constexpr(Instance == 0) {
                    return set(Peripheral::RESETS::Registers<>::RESET::i2c0);
                } else if(Instance == 1) {
                    return set(Peripheral::RESETS::Registers<>::RESET::i2c1);
                }
            }
    }}   // namespace Traits::I2C

    namespace Detail {

        template<typename I2CConfig_>
        struct I2CBase {
            struct I2CConfig : I2CConfig_ {
                static constexpr auto userConfigOverride = [] {
                    if constexpr(requires { I2CConfig_::userConfigOverride; }) {
                        return I2CConfig_::userConfigOverride;
                    } else {
                        return brigand::list<>{};
                    }
                }();

                static constexpr auto maxBaudRateError = [] {
                    if constexpr(requires { I2CConfig_::maxBaudRateError; }) {
                        return I2CConfig_::maxBaudRateError;
                    } else {
                        return std::ratio<1, 100>{};
                    }
                }();
            };

            // needed config
            // clockSpeed
            // baudRate
            // maxBaudRateError
            // i2cInstance
            // SdaPinLocation
            // SclPinLocation
            // userConfigOverride
            static constexpr auto Instance = I2CConfig::instance;
            using Regs                     = Kvasir::Peripheral::I2C::Registers<Instance>;
            using Config                   = Detail::Config<Instance>;

            using InterruptIndexs = decltype(Traits::I2C::getIsrIndexs<Instance>());

            static constexpr auto NoInterrupts = list(Regs::IC_INTR_MASK::overrideDefaults(
              clear(Regs::IC_INTR_MASK::m_gen_call),
              clear(Regs::IC_INTR_MASK::m_rx_done),
              clear(Regs::IC_INTR_MASK::m_tx_abrt),
              clear(Regs::IC_INTR_MASK::m_rd_req),
              clear(Regs::IC_INTR_MASK::m_tx_empty),
              clear(Regs::IC_INTR_MASK::m_tx_over),
              clear(Regs::IC_INTR_MASK::m_rx_full),
              clear(Regs::IC_INTR_MASK::m_rx_over),
              clear(Regs::IC_INTR_MASK::m_rx_under)));

            static constexpr auto TxInterrupts = list(Regs::IC_INTR_MASK::overrideDefaults(
              clear(Regs::IC_INTR_MASK::m_gen_call),
              clear(Regs::IC_INTR_MASK::m_rx_done),
              set(Regs::IC_INTR_MASK::m_tx_abrt),
              clear(Regs::IC_INTR_MASK::m_rd_req),
              set(Regs::IC_INTR_MASK::m_tx_empty),
              clear(Regs::IC_INTR_MASK::m_tx_over),
              clear(Regs::IC_INTR_MASK::m_rx_full),
              clear(Regs::IC_INTR_MASK::m_rx_over),
              clear(Regs::IC_INTR_MASK::m_rx_under)));

            static constexpr auto RxInterrupts = list(Regs::IC_INTR_MASK::overrideDefaults(
              clear(Regs::IC_INTR_MASK::m_gen_call),
              clear(Regs::IC_INTR_MASK::m_rx_done),
              set(Regs::IC_INTR_MASK::m_tx_abrt),
              clear(Regs::IC_INTR_MASK::m_rd_req),
              clear(Regs::IC_INTR_MASK::m_tx_empty),
              clear(Regs::IC_INTR_MASK::m_tx_over),
              set(Regs::IC_INTR_MASK::m_rx_full),
              clear(Regs::IC_INTR_MASK::m_rx_over),
              clear(Regs::IC_INTR_MASK::m_rx_under)));

            static_assert(
              Config::template isValidBaudConfig<I2CConfig::clockSpeed, I2CConfig::baudRate>(
                I2CConfig::maxBaudRateError),
              "invalid baud configuration baudRate error to big");
            static_assert(
              Config::template isValidPinLocationSDA(I2CConfig::sdaPinLocation),
              "invalid SDAPin");
            static_assert(
              Config::template isValidPinLocationSCL(I2CConfig::sclPinLocation),
              "invalid SCLPin");

            static constexpr auto powerClockEnable = list(Traits::I2C::getEnable<Instance>());

            static constexpr auto initStepPinConfig = list(
              typename Config::template GetSDAPinConfig<
                std::decay_t<decltype(I2CConfig::sdaPinLocation)>>::pinConfig{},
              typename Config::template GetSCLPinConfig<
                std::decay_t<decltype(I2CConfig::sclPinLocation)>>::pinConfig{});

            static constexpr auto initStepPeripheryConfig = list(
              Regs::IC_CON::overrideDefaults(write(Regs::IC_CON::MASTER_MODEValC::enabled)),

              //TODO speed type and baud
              write(Regs::IC_FS_SCL_HCNT::ic_fs_scl_hcnt, Kvasir::Register::value<266>()),
              write(Regs::IC_FS_SCL_LCNT::ic_fs_scl_lcnt, Kvasir::Register::value<399>()),
              write(Regs::IC_FS_SPKLEN::ic_fs_spklen, Kvasir::Register::value<24>()),
              Regs::IC_SDA_HOLD::overrideDefaults(
                write(Regs::IC_SDA_HOLD::ic_sda_tx_hold, Kvasir::Register::value<40>())),
              NoInterrupts);

            static constexpr auto initStepInterruptConfig = list(
              Nvic::makeSetPriority<I2CConfig::isrPriority>(InterruptIndexs{}),
              Nvic::makeClearPending(InterruptIndexs{}));

            static constexpr auto initStepPeripheryEnable
              = list(Nvic::makeEnable(InterruptIndexs{}));
        };
    }   // namespace Detail

    template<typename I2CConfig, typename Clock, std::size_t BufferSize_>
    struct I2CBehavior : Detail::I2CBase<I2CConfig> {
        static constexpr std::size_t BufferSize = BufferSize_;
        using base                              = Detail::I2CBase<I2CConfig>;
        using Regs                              = typename base::Regs;
        using tp                                = typename Clock::time_point;

        enum class State { idle, blocked, sending, receiveing };
        enum class OperationState { succeeded, failed, ongoing };
        inline static std::atomic<State>          state_{State::idle};
        inline static std::atomic<std::uint8_t>   receiveSize_{0};
        inline static std::atomic<OperationState> operationState_{OperationState::succeeded};
        inline static Kvasir::Atomic::Queue<std::byte, BufferSize> buffer_{};
        inline static bool                                         stop{false};
        inline static tp                                           timeoutTime{};

        static void reset() {
            apply(makeDisable(typename base::InterruptIndexs{}));
            state_.store(State::idle, std::memory_order_relaxed);
            operationState_.store(OperationState::succeeded, std::memory_order_relaxed);
            apply(Traits::I2C::getDisable<base::Instance>());
            apply(base::powerClockEnable);
            apply(base::initStepPeripheryConfig);
            apply(base::initStepInterruptConfig);
            apply(base::initStepPeripheryEnable);
        }

        template<typename C>
        static void getReceivedBytes(C& c) {
            assert(c.size() <= buffer_.size());
            buffer_.pop_into(c);
        }

        template<typename OIT>
        static void getReceivedBytes(OIT first, OIT last) {
            while(first != last) {
                assert(!buffer_.empty());
                *first = buffer_.front();
                buffer_.pop();
                ++first;
            }
        }

        static OperationState operationState(tp const& currentTime) {
            auto op = operationState_.load(std::memory_order_relaxed);
            if(op == OperationState::ongoing) {
                if(currentTime > timeoutTime) {
                    state_.store(State::blocked, std::memory_order_relaxed);
                    return OperationState::failed;
                }
            }
            return op;
        }

        static bool acquire() {
            if(state_.load(std::memory_order_relaxed) == State::idle) {
                state_.store(State::blocked, std::memory_order_relaxed);
                return true;
            }
            return false;
        }
        static void release() {
            assert(state_.load(std::memory_order_relaxed) != State::idle);   // TODO
            state_.store(State::idle, std::memory_order_relaxed);
        }

        template<typename C>
        static void send(tp const& currentTime, std::uint8_t address, C const& c) {
            assert(state_.load(std::memory_order_relaxed) != State::sending);
            assert(state_.load(std::memory_order_relaxed) != State::receiveing);
            assert(!c.empty());
            assert(c.size() <= buffer_.max_size());

            buffer_.clear();
            buffer_.push(c);
            state_.store(State::sending, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = true;
            timeoutTime = currentTime + 100ms;   // TODO baud*size
            apply(write(Regs::IC_TAR::ic_tar, address));
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled)));
            apply(base::TxInterrupts);
        }

        static void receive(tp const& currentTime, std::uint8_t address, std::uint8_t size) {
            assert(state_.load(std::memory_order_relaxed) != State::sending);
            assert(state_.load(std::memory_order_relaxed) != State::receiveing);
            assert(size <= buffer_.max_size());
            assert(size != 0);

            buffer_.clear();
            receiveSize_.store(size, std::memory_order_relaxed);
            state_.store(State::receiveing, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = true;
            timeoutTime = currentTime + 100ms;   // TODO baud*size

            apply(write(Regs::IC_TAR::ic_tar, address));
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled)));

            if(size == 1) {
                apply(Regs::IC_DATA_CMD::overrideDefaults(
                  write(Regs::IC_DATA_CMD::STOPValC::enable),
                  write(Regs::IC_DATA_CMD::CMDValC::read)));
            } else {
                apply(Regs::IC_DATA_CMD::overrideDefaults(write(Regs::IC_DATA_CMD::CMDValC::read)));
            }

            apply(base::RxInterrupts);
        }

        template<typename C>
        static void
        send_receive(tp const& currentTime, std::uint8_t address, C const& c, std::uint8_t size) {
            assert(state_.load(std::memory_order_relaxed) != State::sending);
            assert(state_.load(std::memory_order_relaxed) != State::receiveing);
            assert(c.size() <= buffer_.max_size());
            assert(!c.empty());
            assert(size <= buffer_.max_size());
            assert(size != 0);

            buffer_.clear();
            buffer_.push(c);
            receiveSize_.store(size, std::memory_order_relaxed);
            state_.store(State::sending, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = false;
            timeoutTime = currentTime + 100ms;   // TODO baud*size

            apply(write(Regs::IC_TAR::ic_tar, address));
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled)));
            apply(base::TxInterrupts);
        }

        static constexpr auto abort = list(
          Regs::IC_ENABLE::overrideDefaults(
            write(Regs::IC_ENABLE::ENABLEValC::disabled),
            set(Regs::IC_ENABLE::abort)),
          base::NoInterrupts);

        // ISR
        static void onIsr() {
            bool const error = get<0>(apply(read(Regs::IC_INTR_STAT::r_tx_abrt)))
                            == Regs::IC_INTR_STAT::R_TX_ABRTVal::active;

            auto lstate  = state_.load(std::memory_order_relaxed);
            auto lostate = operationState_.load(std::memory_order_relaxed);
            // TODO K_ASSERT(lstate != State::blocked && lstate != State::idle);
            // TODO operation state release
            if(lstate == State::sending) {
                if(error) {
                    lstate  = State::blocked;
                    lostate = OperationState::failed;
                    UC_LOG_C("abort send");
                    apply(abort);
                } else {
                    if(!buffer_.empty()) {
                        auto const data = buffer_.front();
                        buffer_.pop();
                        if(buffer_.empty() && stop) {
        /*                    Regs::IC_DATA_CMD::overrideDefaultsRuntime(
                              write(Regs::IC_DATA_CMD::STOPValC::enable),
                              write(Regs::IC_DATA_CMD::dat, static_cast<std::uint8_t>(data)));
*/
                        //TODO fix in kvasir
                            std::uint32_t v = static_cast<std::uint8_t>(data);
                            v = v| 0x200;
                            *reinterpret_cast<std::uint32_t volatile*>(Regs::IC_DATA_CMD::Addr::value)  =v ;

                        } else {
                            Regs::IC_DATA_CMD::overrideDefaultsRuntime(
                              write(Regs::IC_DATA_CMD::dat, static_cast<std::uint8_t>(data)));
                        }
                    } else {
                        if(stop) {
                            lstate  = State::blocked;
                            lostate = OperationState::succeeded;
                            apply(Regs::IC_ENABLE::overrideDefaults(
                              write(Regs::IC_ENABLE::ENABLEValC::disabled)));
                            apply(base::NoInterrupts);
                        } else {
                            auto const lreceiveSize = receiveSize_.load(std::memory_order_relaxed);
                            lstate                  = State::receiveing;
                            if(lreceiveSize == 1) {
                                apply(Regs::IC_DATA_CMD::overrideDefaults(
                                  write(Regs::IC_DATA_CMD::RESTARTValC::enable),
                                  write(Regs::IC_DATA_CMD::STOPValC::enable),
                                  write(Regs::IC_DATA_CMD::CMDValC::read)));
                            } else {
                                apply(Regs::IC_DATA_CMD::overrideDefaults(
                                  write(Regs::IC_DATA_CMD::RESTARTValC::enable),
                                  write(Regs::IC_DATA_CMD::CMDValC::read)));
                            }
                            apply(base::RxInterrupts);
                        }
                    }
                }
            } else if(lstate == State::receiveing) {
                if(error) {
                    UC_LOG_C("abort recv");
                    lstate  = State::blocked;
                    lostate = OperationState::failed;
                    apply(abort);
                } else {
                    auto const lreceiveSize = receiveSize_.load(std::memory_order_relaxed);
                    auto const data         = apply(read(Regs::IC_DATA_CMD::dat));
                    buffer_.push(static_cast<std::byte>(Kvasir::Register::get<0>(data)));
                    auto const bytesLeft = lreceiveSize - buffer_.size();
                    if(bytesLeft > 1) {
                        apply(Regs::IC_DATA_CMD::overrideDefaults(
                          write(Regs::IC_DATA_CMD::CMDValC::read)));
                    } else if(bytesLeft == 1) {
                        apply(Regs::IC_DATA_CMD::overrideDefaults(
                          write(Regs::IC_DATA_CMD::STOPValC::enable),
                          write(Regs::IC_DATA_CMD::CMDValC::read)));
                    } else {
                        lstate  = State::blocked;
                        lostate = OperationState::succeeded;
                        apply(Regs::IC_ENABLE::overrideDefaults(
                          write(Regs::IC_ENABLE::ENABLEValC::disabled)));
                        apply(base::NoInterrupts);
                    }
                }
            } else {
                UC_LOG_C("bad bad");
            }

            operationState_.store(lostate, std::memory_order_relaxed);
            state_.store(lstate, std::memory_order_relaxed);
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }
        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));
    };
}}   // namespace Kvasir::I2C
