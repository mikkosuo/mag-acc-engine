// MagAccEngine.scala -- Chisel integration of rtl/mag_acc_engine.v
//
// Copy rtl/mag_acc_engine.v and rtl/mae_fifo.v to src/main/resources/vsrc/ in the
// Chisel project; HasBlackBoxResource then emits them next to the generated Verilog.
//
// MagAccEngineBlackBox mirrors the Verilog ports one to one. MagAccEngine wraps it
// in a Module with plain AXI4-Lite bundles and the implicit clock and reset; map
// those bundles onto A-Core's own AXI4-Lite types where the engine is attached.
// Not elaborated here (no Maven access when this was written); the Verilog itself
// is verified by the cocotb suite in tb/.

package mag_acc_engine

import chisel3._
import chisel3.experimental.IntParam
import chisel3.util.HasBlackBoxResource

case class MagAccParams(
    nIter: Int     = 11,
    guard: Int     = 6,
    fifoDepth: Int = 32,
    sAddrW: Int    = 8
) {
  require(nIter >= 1 && nIter <= 32, "N_ITER out of range")
  require(guard >= 0 && guard <= 16, "GUARD out of range")
  require(fifoDepth >= 1, "FIFO_DEPTH must be at least 1")
  require(sAddrW >= 5, "the register map needs at least 5 address bits")
}

class MagAccEngineBlackBox(p: MagAccParams)
    extends BlackBox(
      Map(
        "N_ITER"     -> IntParam(p.nIter),
        "GUARD"      -> IntParam(p.guard),
        "FIFO_DEPTH" -> IntParam(p.fifoDepth),
        "S_ADDR_W"   -> IntParam(p.sAddrW)
      )
    )
    with HasBlackBoxResource {

  override def desiredName = "mag_acc_engine"

  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())

    val s_axil_awaddr  = Input(UInt(p.sAddrW.W))
    val s_axil_awprot  = Input(UInt(3.W))
    val s_axil_awvalid = Input(Bool())
    val s_axil_awready = Output(Bool())
    val s_axil_wdata   = Input(UInt(32.W))
    val s_axil_wstrb   = Input(UInt(4.W))
    val s_axil_wvalid  = Input(Bool())
    val s_axil_wready  = Output(Bool())
    val s_axil_bresp   = Output(UInt(2.W))
    val s_axil_bvalid  = Output(Bool())
    val s_axil_bready  = Input(Bool())
    val s_axil_araddr  = Input(UInt(p.sAddrW.W))
    val s_axil_arprot  = Input(UInt(3.W))
    val s_axil_arvalid = Input(Bool())
    val s_axil_arready = Output(Bool())
    val s_axil_rdata   = Output(UInt(32.W))
    val s_axil_rresp   = Output(UInt(2.W))
    val s_axil_rvalid  = Output(Bool())
    val s_axil_rready  = Input(Bool())

    val m_src_araddr  = Output(UInt(32.W))
    val m_src_arprot  = Output(UInt(3.W))
    val m_src_arvalid = Output(Bool())
    val m_src_arready = Input(Bool())
    val m_src_rdata   = Input(UInt(32.W))
    val m_src_rresp   = Input(UInt(2.W))
    val m_src_rvalid  = Input(Bool())
    val m_src_rready  = Output(Bool())

    val m_dst_araddr  = Output(UInt(32.W))
    val m_dst_arprot  = Output(UInt(3.W))
    val m_dst_arvalid = Output(Bool())
    val m_dst_arready = Input(Bool())
    val m_dst_rdata   = Input(UInt(32.W))
    val m_dst_rresp   = Input(UInt(2.W))
    val m_dst_rvalid  = Input(Bool())
    val m_dst_rready  = Output(Bool())
    val m_dst_awaddr  = Output(UInt(32.W))
    val m_dst_awprot  = Output(UInt(3.W))
    val m_dst_awvalid = Output(Bool())
    val m_dst_awready = Input(Bool())
    val m_dst_wdata   = Output(UInt(32.W))
    val m_dst_wstrb   = Output(UInt(4.W))
    val m_dst_wvalid  = Output(Bool())
    val m_dst_wready  = Input(Bool())
    val m_dst_bresp   = Input(UInt(2.W))
    val m_dst_bvalid  = Input(Bool())
    val m_dst_bready  = Output(Bool())

    val irq = Output(Bool())
  })

  addResource("/vsrc/mag_acc_engine.v")
  addResource("/vsrc/mae_fifo.v")
}

// ---------------------------------------------------------------- AXI4-Lite bundles
// Directions are from the point of view of the side that issues requests (master).

class AxiLiteAR(addrW: Int) extends Bundle {
  val araddr  = Output(UInt(addrW.W))
  val arprot  = Output(UInt(3.W))
  val arvalid = Output(Bool())
  val arready = Input(Bool())
  val rdata   = Input(UInt(32.W))
  val rresp   = Input(UInt(2.W))
  val rvalid  = Input(Bool())
  val rready  = Output(Bool())
}

class AxiLiteAW(addrW: Int) extends Bundle {
  val awaddr  = Output(UInt(addrW.W))
  val awprot  = Output(UInt(3.W))
  val awvalid = Output(Bool())
  val awready = Input(Bool())
  val wdata   = Output(UInt(32.W))
  val wstrb   = Output(UInt(4.W))
  val wvalid  = Output(Bool())
  val wready  = Input(Bool())
  val bresp   = Input(UInt(2.W))
  val bvalid  = Input(Bool())
  val bready  = Output(Bool())
}

class AxiLite(addrW: Int) extends Bundle {
  val r = new AxiLiteAR(addrW)
  val w = new AxiLiteAW(addrW)
}

class MagAccEngine(p: MagAccParams = MagAccParams()) extends Module {
  val io = IO(new Bundle {
    val cfg = Flipped(new AxiLite(p.sAddrW)) // the CPU drives the configuration port
    val src = new AxiLiteAR(32)              // X1 reads (buffer bank)
    val dst = new AxiLite(32)                // X2 reads and writes (accumulator bank)
    val irq = Output(Bool())
  })

  val bb = Module(new MagAccEngineBlackBox(p))
  bb.io.clock := clock
  bb.io.reset := reset.asBool

  // configuration slave
  bb.io.s_axil_awaddr  := io.cfg.w.awaddr
  bb.io.s_axil_awprot  := io.cfg.w.awprot
  bb.io.s_axil_awvalid := io.cfg.w.awvalid
  io.cfg.w.awready     := bb.io.s_axil_awready
  bb.io.s_axil_wdata   := io.cfg.w.wdata
  bb.io.s_axil_wstrb   := io.cfg.w.wstrb
  bb.io.s_axil_wvalid  := io.cfg.w.wvalid
  io.cfg.w.wready      := bb.io.s_axil_wready
  io.cfg.w.bresp       := bb.io.s_axil_bresp
  io.cfg.w.bvalid      := bb.io.s_axil_bvalid
  bb.io.s_axil_bready  := io.cfg.w.bready
  bb.io.s_axil_araddr  := io.cfg.r.araddr
  bb.io.s_axil_arprot  := io.cfg.r.arprot
  bb.io.s_axil_arvalid := io.cfg.r.arvalid
  io.cfg.r.arready     := bb.io.s_axil_arready
  io.cfg.r.rdata       := bb.io.s_axil_rdata
  io.cfg.r.rresp       := bb.io.s_axil_rresp
  io.cfg.r.rvalid      := bb.io.s_axil_rvalid
  bb.io.s_axil_rready  := io.cfg.r.rready

  // source master (read only)
  io.src.araddr        := bb.io.m_src_araddr
  io.src.arprot        := bb.io.m_src_arprot
  io.src.arvalid       := bb.io.m_src_arvalid
  bb.io.m_src_arready  := io.src.arready
  bb.io.m_src_rdata    := io.src.rdata
  bb.io.m_src_rresp    := io.src.rresp
  bb.io.m_src_rvalid   := io.src.rvalid
  io.src.rready        := bb.io.m_src_rready

  // destination master
  io.dst.r.araddr      := bb.io.m_dst_araddr
  io.dst.r.arprot      := bb.io.m_dst_arprot
  io.dst.r.arvalid     := bb.io.m_dst_arvalid
  bb.io.m_dst_arready  := io.dst.r.arready
  bb.io.m_dst_rdata    := io.dst.r.rdata
  bb.io.m_dst_rresp    := io.dst.r.rresp
  bb.io.m_dst_rvalid   := io.dst.r.rvalid
  io.dst.r.rready      := bb.io.m_dst_rready
  io.dst.w.awaddr      := bb.io.m_dst_awaddr
  io.dst.w.awprot      := bb.io.m_dst_awprot
  io.dst.w.awvalid     := bb.io.m_dst_awvalid
  bb.io.m_dst_awready  := io.dst.w.awready
  io.dst.w.wdata       := bb.io.m_dst_wdata
  io.dst.w.wstrb       := bb.io.m_dst_wstrb
  io.dst.w.wvalid      := bb.io.m_dst_wvalid
  bb.io.m_dst_wready   := io.dst.w.wready
  bb.io.m_dst_bresp    := io.dst.w.bresp
  bb.io.m_dst_bvalid   := io.dst.w.bvalid
  io.dst.w.bready      := bb.io.m_dst_bready

  io.irq := bb.io.irq
}
