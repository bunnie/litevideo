from migen import *
from migen.genlib.fsm import FSM, NextState

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.interconnect import stream

from litedram.frontend.dma import LiteDRAMDMAWriter

# Slot status: EMPTY=0 LOADED=1 PENDING=2
# Slot stores the data associated with a slot
# CPU writes 0x1 to status to indicate the slot is loaded
# FPGA updates status to 0x2 when address_done is hit
# The internal update to 0x2 also triggers an interrupt:
# Note this means you can self-interrupt if the CPU sets this bit!
class _Slot(Module, AutoCSR):
    def __init__(self, addr_bits, alignment_bits):
#        self.ev_source = EventSourcePulse()
        self.address = Signal(addr_bits)
        self.address_reached = Signal(addr_bits)
        self.address_valid = Signal()
        self.address_done = Signal()

        self._status = CSRStorage(2, write_from_dev=True)
        self._address = CSRStorage(addr_bits + alignment_bits,
                                   alignment_bits=alignment_bits,
                                   write_from_dev=True)

        # # #

        self.comb += [
            self.address.eq(self._address.storage),
            self.address_valid.eq(self._status.storage[0]),
            self._status.dat_w.eq(2),
            self._status.we.eq(self.address_done),
            self._address.dat_w.eq(self.address_reached),
            self._address.we.eq(self.address_done),
#            self.ev_source.trigger.eq(self._status.storage[1])
#            self.ev_source.trigger.eq(0)  # disable interrupts for now
        ]


class _SlotArray(Module, AutoCSR):
    def __init__(self, nslots, addr_bits, alignment_bits):
#        self.submodules.ev = EventManager()
        self.address = Signal(addr_bits)
        self.address_reached = Signal(addr_bits)
        self.address_valid = Signal()
        self.address_done = Signal()

        # # #

        slots = [_Slot(addr_bits, alignment_bits) for i in range(nslots)]
        for n, slot in enumerate(slots):
            setattr(self.submodules, "slot"+str(n), slot)
#            setattr(self.ev, "slot"+str(n), slot.ev_source)
#        self.ev.finalize()

        change_slot = Signal()
        current_slot = Signal(max=nslots)
        # creates a list, sorted from highest to lowest, of If(slot.address_valid, current_slot.eq(n))
        # and processes the list only if change_slot is true
        # the list is computed inside a self.sync block, which means if multiple address_valid is true,
        # the item on the *bottom* of the list takes precedence (due to the behavior of the verilog <= operator)
        # which is why the "reversed()" is important: it ensures that the slot with the lowest number
        # and with a valid address is the slot that gets picked.
        self.sync += If(change_slot,
                        [If(slot.address_valid, current_slot.eq(n))
                         for n, slot in reversed(list(enumerate(slots)))]
                        )
        self.comb += change_slot.eq(~self.address_valid | self.address_done)

        # pick the address and address_valid based on the current_slot indicated from external signals
        self.comb += [
            self.address.eq(Array(slot.address for slot in slots)[current_slot]),
            self.address_valid.eq(Array(slot.address_valid for slot in slots)[current_slot])
        ]

        # assign address_reached from external signals to each of the _Slot
        self.comb += [slot.address_reached.eq(self.address_reached) for slot in slots]

        # assign address_done from external address_done to the _Slot, but only if the current_slot matches the slot number
        self.comb += [slot.address_done.eq(self.address_done & (current_slot == n))
                          for n, slot in enumerate(slots)]


class DMA_poll(Module, AutoCSR):
    def __init__(self, dram_port, nslots):
        bus_aw = dram_port.aw
        bus_dw = dram_port.dw
        alignment_bits = bits_for(bus_dw//8) - 1

        fifo_word_width = bus_dw
        self.frame = stream.Endpoint([("sof", 1), ("pixels", fifo_word_width)])
        self._frame_size = CSRStorage(bus_aw + alignment_bits,
                                      alignment_bits=alignment_bits)

        self._current_address = CSRStorage(bus_aw + alignment_bits, alignment_bits=alignment_bits)

        self._start_address = CSRStorage(bus_aw + alignment_bits, alignment_bits=alignment_bits)
        self.start_address = Signal(bus_aw)
        self.comb += self.start_address.eq(self._start_address.storage)

        self.address_valid = CSRStorage()
        self.dma_running = CSRStatus()
        self.last_count_reached = CSRStatus(bus_aw + alignment_bits)
        self.last_count_snapshot = Signal()
        self.last_count_holding = Signal(bus_aw + alignment_bits)
        self._dma_go = CSRStorage()
        self.dma_go = Signal()

        # address generator + maximum memory word count to prevent DMA buffer
        # overrun
        self.reset_words = reset_words = Signal()
        self.count_word = count_word = Signal()
        self.last_word = last_word = Signal()
        self.current_address = current_address = Signal(bus_aw)
        self.mwords_remaining = mwords_remaining = Signal(bus_aw)
        self.comb += [
            self._current_address.storage.eq(current_address),
            last_word.eq(mwords_remaining == 1)
        ]
        last_count_snapshot_r = Signal()
        self.sync += [
            If(reset_words,
                current_address.eq(self.start_address),
                mwords_remaining.eq(self._frame_size.storage)
            ).Elif(count_word,
                current_address.eq(current_address + 1),
                mwords_remaining.eq(mwords_remaining - 1)
            ),

            last_count_snapshot_r.eq(self.last_count_snapshot),
            If(self.last_count_snapshot & ~last_count_snapshot_r,
               self.last_count_holding.eq(current_address)
            ).Else(
                self.last_count_holding.eq(self.last_count_holding)
            ),

            If(self._dma_go.re,
               self.dma_go.eq(1)
            ).Elif(last_word,
                self.dma_go.eq(0)
            ).Else(
                self.dma_go.eq(self.dma_go)
            )
        ]
        self.comb += self.last_count_reached.status.eq(self.last_count_holding)

        memory_word = Signal(bus_dw)
        pixbits = []
        for i in range(bus_dw//16):
            pixbits.append(self.frame.pixels)
        self.comb += memory_word.eq(Cat(*pixbits))

        # bus accessor
        self.submodules._bus_accessor = LiteDRAMDMAWriter(dram_port)
        self.comb += [
            self._bus_accessor.sink.address.eq(current_address),
            self._bus_accessor.sink.data.eq(memory_word)
        ]

        # control FSM
        self.fsm = fsm = FSM()
        self.submodules += fsm

        fsm.act("WAIT_SOF",
            reset_words.eq(1),
                self.frame.ready.eq(~self.address_valid.storage |
                                ~self.frame.sof),
            If(self.address_valid.storage &
               self.frame.sof &
               self.frame.valid &
               self.dma_go,
               NextState("TRANSFER_PIXELS")
            )
        )
        fsm.act("TRANSFER_PIXELS",
            self.dma_running.status.eq(1),
            self.frame.ready.eq(self._bus_accessor.sink.ready),
            If(self.frame.valid,
                self._bus_accessor.sink.valid.eq(1),
                If(self._bus_accessor.sink.ready,
                    count_word.eq(1),
                    If(last_word,
                        NextState("EOF")
                    )
                )
            )
        )
        fsm.act("EOF",
            If(~dram_port.wdata.valid,
                self.last_count_snapshot.eq(1),
                NextState("WAIT_SOF")
            )
        )

class DMA(Module):
        def __init__(self, dram_port, nslots):
            bus_aw = dram_port.aw
            bus_dw = dram_port.dw
            alignment_bits = bits_for(bus_dw // 8) - 1

            fifo_word_width = bus_dw
            self.frame = stream.Endpoint([("sof", 1), ("pixels", fifo_word_width)])
            self._frame_size = CSRStorage(bus_aw + alignment_bits,
                                          alignment_bits=alignment_bits)
            self.submodules._slot_array = _SlotArray(nslots, bus_aw, alignment_bits)
            #        self.ev = self._slot_array.ev

            # # #

            # address generator + maximum memory word count to prevent DMA buffer
            # overrun
            reset_words = Signal()
            count_word = Signal()
            last_word = Signal()
            current_address = Signal(bus_aw)
            mwords_remaining = Signal(bus_aw)
            self.comb += [
                self._slot_array.address_reached.eq(current_address),
                last_word.eq(mwords_remaining == 1)
            ]
            self.sync += [
                If(reset_words,
                   current_address.eq(self._slot_array.address),
                   mwords_remaining.eq(self._frame_size.storage)
                   ).Elif(count_word,
                          current_address.eq(current_address + 1),
                          mwords_remaining.eq(mwords_remaining - 1)
                          )
            ]

            memory_word = Signal(bus_dw)
            pixbits = []
            for i in range(bus_dw // 16):
                pixbits.append(self.frame.pixels)
            self.comb += memory_word.eq(Cat(*pixbits))

            # bus accessor
            self.submodules._bus_accessor = LiteDRAMDMAWriter(dram_port)
            self.comb += [
                self._bus_accessor.sink.address.eq(current_address),
                self._bus_accessor.sink.data.eq(memory_word)
            ]

            # control FSM
            self.fsm = fsm = FSM()
            self.submodules += fsm

            fsm.act("WAIT_SOF",
                    reset_words.eq(1),
                    self.frame.ready.eq(~self._slot_array.address_valid |
                                        ~self.frame.sof),
                    If(self._slot_array.address_valid &
                       self.frame.sof &
                       self.frame.valid,
                       NextState("TRANSFER_PIXELS")
                       )
                    )
            fsm.act("TRANSFER_PIXELS",
                    self.frame.ready.eq(self._bus_accessor.sink.ready),
                    If(self.frame.valid,
                       self._bus_accessor.sink.valid.eq(1),
                       If(self._bus_accessor.sink.ready,
                          count_word.eq(1),
                          If(last_word,
                             NextState("EOF")
                             )
                          )
                       )
                    )
            fsm.act("EOF",
                    If(~dram_port.wdata.valid,
                       self._slot_array.address_done.eq(1),
                       NextState("WAIT_SOF")
                       )
                    )

#def get_csrs(self):
#        return [self._frame_size] + self._slot_array.get_csrs()
