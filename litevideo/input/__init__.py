from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR

from litevideo.input.edid import EDID, _default_edid
from litevideo.input.clocking import S6Clocking, S7Clocking
from litevideo.input.datacapture import S6DataCapture, S7DataCapture
from litevideo.input.charsync import CharSync
from litevideo.input.wer import WER
from litevideo.input.decoding import Decoding
from litevideo.input.chansync import ChanSync
from litevideo.input.analysis import SyncPolarity, ResolutionDetection
from litevideo.input.analysis import FrameExtraction
from litevideo.input.dma import DMA

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

clocking_cls = {
    "xc6" : S6Clocking,
    "xc7" : S7Clocking,
}

datacapture_cls = {
    "xc6" : S6DataCapture,
    "xc7" : S7DataCapture
}


class HDMIIn(Module, AutoCSR):
    def __init__(self, pads, dram_port=None, n_dma_slots=2, fifo_depth=512, device="xc6",
                 default_edid=_default_edid, clkin_freq=148.5e6, bypass=False):
        if hasattr(pads, "scl"):
            self.submodules.edid = EDID(pads, default_edid)
        self.submodules.clocking = clocking_cls[device](pads, clkin_freq)

        for datan in range(3):
            name = "data" + str(datan)

            cap = datacapture_cls[device](getattr(pads, name + "_p"),
                                          getattr(pads, name + "_n"))
            setattr(self.submodules, name + "_cap", cap)
            if hasattr(cap, "serdesstrobe"):
                self.comb += cap.serdesstrobe.eq(self.clocking.serdesstrobe)

            charsync = CharSync()
            setattr(self.submodules, name + "_charsync", charsync)
            self.comb += charsync.raw_data.eq(cap.d)

            wer = WER()
            setattr(self.submodules, name + "_wer", wer)
            self.comb += wer.data.eq(charsync.data)

            decoding = Decoding()
            setattr(self.submodules, name + "_decod", decoding)
            self.comb += [
                decoding.valid_i.eq(charsync.synced),
                decoding.input.eq(charsync.data)
            ]

        if not bypass: 
            self.submodules.chansync = ChanSync()
            self.comb += [
                self.chansync.valid_i.eq(self.data0_decod.valid_o &
                                         self.data1_decod.valid_o &
                                         self.data2_decod.valid_o),
                self.chansync.data_in0.eq(self.data0_decod.output),
                self.chansync.data_in1.eq(self.data1_decod.output),
                self.chansync.data_in2.eq(self.data2_decod.output)
            ]

            self.submodules.syncpol = SyncPolarity()
            self.comb += [
                self.syncpol.valid_i.eq(self.chansync.chan_synced),
                self.syncpol.data_in0.eq(self.chansync.data_out0),
                self.syncpol.data_in1.eq(self.chansync.data_out1),
                self.syncpol.data_in2.eq(self.chansync.data_out2)
            ]
            
            self.submodules.resdetection = ResolutionDetection()
            self.comb += [
                self.resdetection.valid_i.eq(self.syncpol.valid_o),
                self.resdetection.de.eq(self.syncpol.de),
                self.resdetection.vsync.eq(self.syncpol.vsync)
            ]

            if dram_port is not None:
                self.submodules.frame = FrameExtraction(dram_port.dw, fifo_depth)
                self.comb += [
                    self.frame.valid_i.eq(self.syncpol.valid_o),
                    self.frame.de.eq(self.syncpol.de),
                    self.frame.vsync.eq(self.syncpol.vsync),
                    self.frame.r.eq(self.syncpol.r),
                    self.frame.g.eq(self.syncpol.g),
                    self.frame.b.eq(self.syncpol.b)
                ]


                self.submodules.dma = DMA(dram_port, n_dma_slots)
                self.comb += self.frame.frame.connect(self.dma.frame)
                self.ev = self.dma.ev
        else:
            self.submodules.chansync = ChanSync()
            self.comb += [
                self.chansync.valid_i.eq(self.data0_decod.valid_o &
                                         self.data1_decod.valid_o &
                                         self.data2_decod.valid_o),
                self.chansync.data_in0.eq(self.data0_decod.output),
                self.chansync.data_in1.eq(self.data1_decod.output),
                self.chansync.data_in2.eq(self.data2_decod.output)
            ]

            self.submodules.syncpol = SyncPolarity()
            self.comb += [
                self.syncpol.valid_i.eq(self.chansync.chan_synced),
                self.syncpol.data_in0.eq(self.chansync.data_out0),
                self.syncpol.data_in1.eq(self.chansync.data_out1),
                self.syncpol.data_in2.eq(self.chansync.data_out2)
            ]
            
            self.hdmi_in0_reset = ResetSignal("hdmi_in0_pix")
            self.sdout = Signal(30)
            self.hi0_red = Signal(8)
            self.hi0_green = Signal(8)
            self.hi0_blue = Signal(8)
            self.hi0_vsync = Signal()
            self.hi0_hsync = Signal()
            self.hi0_valid = Signal()
            self.hi0_de = Signal()
            self.hi0_ctl = Signal(4)
            self.hi0_video_gb = Signal()
            self.hi0_basic_de = Signal()
            self.specials += Instance("hdmi_decoder",
                                      i_p_clk=self.clocking.cd_pix.clk,
                                      i_reset_in=self.hdmi_in0_reset,
                                      i_raw_b=self.data0_charsync.data,
                                      i_raw_g=self.data1_charsync.data,
                                      i_raw_r=self.data2_charsync.data,
                                      i_vld_b=self.data0_charsync.synced,
                                      i_vld_g=self.data1_charsync.synced,
                                      i_vld_r=self.data2_charsync.synced,
                                      o_sdout=self.sdout,
                                      o_red=self.hi0_red,
                                      o_green=self.hi0_green,
                                      o_blue=self.hi0_blue,
                                      o_vsync=self.hi0_vsync,
                                      o_hsync=self.hi0_hsync,
                                      o_valid=self.hi0_valid,
                                      o_de=self.hi0_de,
                                      o_ctl_code=self.hi0_ctl,
                                      o_video_gb=self.hi0_video_gb,
                                      o_basic_de=self.hi0_basic_de,
            )
            
            self.submodules.resdetection = ResolutionDetection()
            self.comb += [
                self.resdetection.valid_i.eq(self.hi0_valid),
                self.resdetection.de.eq(self.hi0_de),
                self.resdetection.vsync.eq(self.hi0_vsync)
            ]

            # establish stream format
            word_layout = [("sof", 1), ("pixels", 32)]
            self.frame = stream.Endpoint(word_layout)
            self.busy = Signal()
            
            # start of frame detection
            vsync_r = Signal()
            new_frame = Signal()
            self.comb += new_frame.eq(self.hi0_vsync & ~vsync_r)
            self.sync.pix += vsync_r.eq(self.hi0_vsync)
            
            # FIFO
            dummy = Signal(2)
            self.comb += dummy.eq(0)
            
            fifo = stream.AsyncFIFO(word_layout, fifo_depth)
            fifo = ClockDomainsRenamer({"write": "pix", "read": "sys"})(fifo)
            self.submodules += fifo
            self.comb += [
                fifo.sink.pixels.eq(Cat(self.sdout,dummy)),  # puts dummy on the MSBs
                fifo.sink.valid.eq(self.hi0_valid)
            ]
            self.sync.pix += \
                             If(new_frame,
                                fifo.sink.sof.eq(1)
                             ).Elif(self.hi0_valid,
                                    fifo.sink.sof.eq(0)
                             )

            self.comb += [
                fifo.source.connect(self.frame),
                self.busy.eq(0)
            ]

            self.submodules.dma = DMA(dram_port, n_dma_slots)
            self.comb += self.frame.connect(self.dma.frame)
            self.ev = self.dma.ev

    autocsr_exclude = {"ev"}
