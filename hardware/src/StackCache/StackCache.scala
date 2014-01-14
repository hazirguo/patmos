/*
   Copyright 2013 Technical University of Denmark, DTU Compute. 
   All rights reserved.
   
   This file is part of the time-predictable VLIW processor Patmos.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

      1. Redistributions of source code must retain the above copyright notice,
         this list of conditions and the following disclaimer.

      2. Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER ``AS IS'' AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
   NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   The views and conclusions contained in the software and documentation are
   those of the authors and should not be interpreted as representing official
   policies, either expressed or implied, of the copyright holder.
 */

/*
 * Stack cache memory 
 * 
 * Author: Sahar Abbaspour (sabb@dtu.dk)
 * 
 */

package stackcache

import Chisel._
import Node._

import scala.math

import ocp._
import patmos._
import patmos.Constants._


class StackCache(SCACHE_SIZE: Int, burstLen: Int) extends Module {
  val io = new Bundle {
    val master = new OcpCoreSlavePort(EXTMEM_ADDR_WIDTH, DATA_WIDTH) // slave to cpu
    val slave = new OcpBurstMasterPort(EXTMEM_ADDR_WIDTH, DATA_WIDTH, burstLen) // master to memory
    val scIO = new StackCacheIO()
  }

  val memReg = Reg(new DecScEx(), init = DecScExResetVal)
  memReg := io.scIO.exsc.decscex

  val mTopReg = Reg(init = UInt(0, width = ADDR_WIDTH))
  mTopReg := io.scIO.exsc.mTop
  
  val slaveCmdReg = Reg(init = Bits(0, width = 3))
  val slaveDataValidReg = Reg(init = Bits(0, width = 1))
  
  val stall_ = Reg(init = Bits(0,1))
  

  
  val sc0 = Mem(Bits(width = BYTE_WIDTH), SCACHE_SIZE)
  val sc1 = Mem(Bits(width = BYTE_WIDTH), SCACHE_SIZE)
  val sc2 = Mem(Bits(width = BYTE_WIDTH), SCACHE_SIZE)
  val sc3 = Mem(Bits(width = BYTE_WIDTH), SCACHE_SIZE)

  val init_st :: spill_st :: fill_st :: free_st :: Nil = Enum(Bits(), 4)
  val state = Reg(init = init_st)
  val nSpill = Reg(init = SInt(1, width = log2Up(SCACHE_SIZE)))
  val nFill = Reg(init = SInt(1, width = log2Up(SCACHE_SIZE)))
  val m_top = Reg(init = mTopReg) // 
  val spill = Reg(init = UInt(0, 1))
  val fill = Reg(init = UInt(0, 1))
  val fill_en = Mux(state === fill_st, UInt(15), UInt(0))

  val addrUFix = EXTMEM_ADDR_WIDTH //log2Up(mem_size)
  val sc_en = Mux(io.master.M.Cmd === OcpCmd.WR, io.master.M.ByteEn, UInt(0))

  val SC_MASK = UInt(width = ADDR_WIDTH)
  SC_MASK := (UInt(SCACHE_SIZE) << UInt(2)) - UInt(1)

  //    val first_addr = Reg(resetVal = Bits(0, width = ADDR_WIDTH))
  //    val first_cmd = Reg(resetVal = Bits(0, 3))
  //    val first_data = Reg(resetVal = Bits(0, width = DATA_WIDTH))

  val scLdStAddr = UInt(width = ADDR_WIDTH)
  scLdStAddr := io.master.M.Addr 
  val cpu_addr_masked = UInt(width = ADDR_WIDTH)
  cpu_addr_masked := (scLdStAddr & SC_MASK)(addrUFix + 1, 2)
  // val cpu_addr_masked = scLdStAddr(addrUFix + 1, 2)

  val spillEnCnt = Reg(init = SInt(0, width = log2Up(SCACHE_SIZE)))
  val fillEnCnt = Reg(init = SInt(0, width = log2Up(SCACHE_SIZE)))

  val rdData = Reg(Bits())

  val sResp = Reg(init = Bits(0, 2))

  val mTopAlign = m_top - UInt(3)
  val FillAlignCnt = UInt(burstLen) - mTopAlign(burstLen - 1, log2Up(burstLen))
  val alignedFillAddr = m_top - (FillAlignCnt << UInt(2)) // alignment of m_top for fill

  val fillCnt = SInt(width = ADDR_WIDTH)
  fillCnt := memReg.nFill
  val alignFillCnt = SInt(width = ADDR_WIDTH)
  alignFillCnt := fillCnt + FillAlignCnt

  val mTopDec = m_top - UInt(4) // --m_top
  val mTopDecAlign = mTopDec + UInt(1)
  val alignCnt = UInt(burstLen) - mTopDec(burstLen - 1, log2Up(burstLen))
  val alignedAddr = mTopDec + (alignCnt << UInt(2)) // alignment of m_top for spill

  val spillCnt = SInt(width = ADDR_WIDTH)
  spillCnt := memReg.nSpill
  val alignSpillCnt = SInt(width = ADDR_WIDTH)
  alignSpillCnt := spillCnt + alignCnt

  val moduluSpillCnt = spillCnt(log2Up(burstLen) - 1, 0).toUInt

  val memAddrReg = Reg(init = Bits(0, width = ADDR_WIDTH))
  memAddrReg := m_top

  val spillAddr = Mux(io.slave.S.DataAccept === UInt(1), (m_top - UInt(4)) & SC_MASK, m_top & SC_MASK)
  val mem_addr_masked = spillAddr(addrUFix + 1, 2) // load from stack cache when spilling

  val ldAddress = Mux(spill === UInt(1) || memReg.spill === Bits(1), mem_addr_masked, cpu_addr_masked)
  rdData := Cat(sc3(ldAddress),
    sc2(ldAddress),
    sc1(ldAddress),
    sc0(ldAddress))

  io.slave.M.Data := rdData
  io.master.S.Data := rdData

  sResp := Mux(io.master.M.Cmd === OcpCmd.WR || io.master.M.Cmd === OcpCmd.RD, OcpResp.DVA, OcpResp.NULL)
  io.master.S.Resp := sResp

  io.slave.M.Cmd := OcpCmd.IDLE
  io.slave.M.Addr := UInt(0)
  io.slave.M.Data := UInt(0)
  io.slave.M.DataByteEn := UInt(0)
  io.slave.M.DataValid := UInt(0)

  io.scIO.stall := stall_//UInt(1)
  io.scIO.memscdec.mTop := m_top

  when(state === init_st) {
    // spill
    when(memReg.spill === Bits(1)) {
      stall_ := UInt(0)
      spill := UInt(1)
      nSpill := alignSpillCnt
      spillEnCnt := SInt(burstLen) - moduluSpillCnt
      m_top := alignedAddr
      state := spill_st
    }

      // fill
      .elsewhen(memReg.fill === Bits(1)) {
        stall_ := UInt(0)
        fill := UInt(1)
        m_top := alignedFillAddr
        nFill := alignFillCnt
        fillEnCnt := SInt(burstLen) - fillCnt(log2Up(burstLen) - 1, 0)
        state := fill_st
      }
      // free
      .elsewhen(memReg.free === Bits(1)) {
        state := free_st
        stall_ := UInt(1)
      }
  }

  when(state === spill_st) {
    stall_ := UInt(0)
    slaveCmdReg := OcpCmd.WR
    io.slave.M.Addr := m_top
    io.slave.M.Data := rdData

    when(io.slave.S.DataAccept === UInt(1)) { //start transfer
      when(nSpill - SInt(2) >= SInt(0)) {
        slaveDataValidReg := Mux(spillEnCnt === SInt(0), UInt(1), UInt(0))
        m_top := m_top - UInt(4) // --m_top
        nSpill := nSpill - UInt(1)
        spillEnCnt := Mux(spillEnCnt === SInt(0), spillEnCnt, spillEnCnt - SInt(1))
        io.slave.M.DataByteEn := Mux(spillEnCnt === SInt(0), UInt(15), UInt(0))
        state := spill_st
      }
        .otherwise {
          slaveCmdReg := OcpCmd.IDLE
          state := init_st
          spill := UInt(0)
        }

    }

  }

  val fillEn = Mux(fillEnCnt === SInt(0), UInt(1), UInt(0))
  val stData = Mux(state === fill_st, io.slave.S.Data, io.master.M.Data)
  val stAddr = Mux(fill === UInt(1) || memReg.fill === Bits(1), (m_top & SC_MASK)(addrUFix + 1, 2), cpu_addr_masked)
  val scEn = Mux(state === fill_st, fillEn, sc_en)

  when(state === fill_st) {
    stall_ := UInt(0)
    slaveCmdReg := OcpCmd.RD
    io.slave.M.Addr := m_top

    when(io.slave.S.Resp === OcpResp.DVA) { // 

      when((nFill - SInt(2)) >= SInt(0)) {
        slaveCmdReg := OcpCmd.IDLE

        m_top := m_top + UInt(4)
        nFill := nFill - UInt(1)
        when(fillEnCnt - SInt(1) >= SInt(0)) { fillEnCnt := fillEnCnt - SInt(1) }
      }
        .otherwise {
          slaveCmdReg := OcpCmd.IDLE
          fill := UInt(0)
          state := init_st
          stall_ := UInt(1)

        }
    }
  }

  when(scEn(0)) { sc0(stAddr) := stData(BYTE_WIDTH - 1, 0) }
  when(scEn(1)) { sc1(stAddr) := stData(2 * BYTE_WIDTH - 1, BYTE_WIDTH) }
  when(scEn(2)) { sc2(stAddr) := stData(3 * BYTE_WIDTH - 1, 2 * BYTE_WIDTH) }
  when(scEn(3)) { sc3(stAddr) := stData(DATA_WIDTH - 1, 3 * BYTE_WIDTH) }

  when(state === free_st) {
    when(memReg.sp > m_top) {
      m_top := memReg.sp
    }
    stall_ := UInt(0)
    when(memReg.fill === UInt(1)) {
      stall_ := UInt(1)
      fill := UInt(1)
      m_top := alignedFillAddr
      nFill := alignFillCnt
      fillEnCnt := SInt(burstLen) - fillCnt(log2Up(burstLen) - 1, 0)
      slaveCmdReg := OcpCmd.RD
      io.slave.M.Addr := m_top
      state := fill_st
    }
      .elsewhen(memReg.spill === UInt(1)) {

        state := spill_st
      }
      .otherwise { state := init_st }
  }

  io.scIO.stall := stall_
  io.slave.M.Cmd := slaveCmdReg
  io.slave.M.DataValid := slaveDataValidReg
}
  





