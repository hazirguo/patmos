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
 * Decode stage of Patmos.
 * 
 * Authors: Martin Schoeberl (martin@jopdesign.com)
 *          Wolfgang Puffitsch (wpuffitsch@gmail.com)
 */

package patmos

import Chisel._
import Node._

import Constants._

class Decode() extends Module {
  val io = new DecodeIO()

  val rf = Module(new RegisterFile())

  // register file is connected with unregistered instruction word
  rf.io.rfRead.rsAddr(0) := io.fedec.instr_a(16, 12)
  rf.io.rfRead.rsAddr(1) := io.fedec.instr_a(11, 7)
  if (PIPE_COUNT > 1) {
	rf.io.rfRead.rsAddr(2) := io.fedec.instr_b(16, 12)
	rf.io.rfRead.rsAddr(3) := io.fedec.instr_b(11, 7)
  }
  rf.io.ena := io.ena
  // RF write from write back stage
  rf.io.rfWrite <> io.rfWrite

  // register input from fetch stage
  val decReg = Reg(new FeDec(), init = FeDecResetVal)
  when(io.ena) {
    decReg := io.fedec
  }

  // forward RF addresses and data
  io.decex.rsAddr(0) := decReg.instr_a(16, 12)
  io.decex.rsAddr(1) := decReg.instr_a(11, 7)
  if (PIPE_COUNT > 1) {
	io.decex.rsAddr(2) := decReg.instr_b(16, 12)
	io.decex.rsAddr(3) := decReg.instr_b(11, 7)
  }

  io.decex.rsData(0) := rf.io.rfRead.rsData(0)
  io.decex.rsData(1) := rf.io.rfRead.rsData(1)
  if (PIPE_COUNT > 1) {
	io.decex.rsData(2) := rf.io.rfRead.rsData(2)
	io.decex.rsData(3) := rf.io.rfRead.rsData(3)
  }
  

  // Decoding of dual-issue operations
  val dual = decReg.instr_a(INSTR_WIDTH-1) && decReg.instr_a(26, 22) != OPCODE_ALUL;
  for (i <- 0 until PIPE_COUNT) {
	val instr   = if (i == 0) { decReg.instr_a } else { decReg.instr_b }
	val opcode  = instr(26, 22)
	val opc     = instr(6, 4)
	val isValid = if (i == 0) { Bool(true) } else { dual }

	// Start with some useful defaults
	io.decex.immOp(i) := Bool(false)
	io.decex.aluOp(i).isMul := Bool(false)
	io.decex.aluOp(i).isCmp := Bool(false)
	io.decex.aluOp(i).isPred := Bool(false)
	io.decex.aluOp(i).isMTS := Bool(false)
	io.decex.aluOp(i).isMFS := Bool(false)
	io.decex.aluOp(i).isSTC := Bool(false)
	io.decex.wrRd(i) := Bool(false)

	// ALU register
	io.decex.aluOp(i).func := instr(3, 0)

	// ALU immediate
	when(opcode(4, 3) === OPCODE_ALUI) {
      io.decex.aluOp(i).func := Cat(Bits(0), instr(24, 22))
      io.decex.immOp(i) := isValid
      io.decex.wrRd(i) := isValid
	}
	// Other ALU
	when(opcode === OPCODE_ALU) {
      switch(opc) {
		is(OPC_ALUR) { io.decex.wrRd(i) := isValid }
		is(OPC_ALUU) { io.decex.wrRd(i) := isValid }
		is(OPC_ALUM) { io.decex.aluOp(i).isMul := isValid }
		is(OPC_ALUC) { io.decex.aluOp(i).isCmp := isValid }
		is(OPC_ALUP) { io.decex.aluOp(i).isPred := isValid }
      }
	}
	// Special registers
	when(opcode === OPCODE_SPC) {
	  switch(opc) {
		is(OPC_MTS) {
		  io.decex.aluOp(i).isMTS := isValid
		}
		is(OPC_MFS) {
		  io.decex.aluOp(i).isMFS := isValid
		  io.decex.wrRd(i) := isValid
		}
	  }
	}

	// Default immediate value
	io.decex.immVal(i) := Cat(Bits(0), instr(11, 0))

	// Predicates
	io.decex.predOp(i).func := Cat(instr(3), instr(0))
	io.decex.predOp(i).s1Addr := instr(15, 12)
	io.decex.predOp(i).s2Addr := instr(10, 7)
	io.decex.predOp(i).dest := instr(19, 17)
	io.decex.pred(i) := instr(30, 27)

	// Default destination
	io.decex.rdAddr(i) := instr(21, 17)
  }

  // Decoding of additional operations for first pipeline
  val instr   = decReg.instr_a
  val opcode  = instr(26, 22)
  val func    = instr(3, 0)

  val ldsize  = instr(11, 9)
  val ldtype  = instr(8, 7)
  val stsize  = instr(21, 19)
  val sttype  = instr(18, 17)
  val stcfun  = instr(21, 18)

  val dest    = Bits(width = REG_BITS)
  val longImm = Bool()

  val isMem   = Bool()
  val isStack = Bool()

  val isSTC   = Bool()
  val stcVal  = Bits(width = DATA_WIDTH)
  val stcImm  = Cat(Bits(0), instr(17, 0), Bits("b00")).toUInt()

  io.decex.jmpOp.branch := Bool(false)
  io.decex.memOp.load := Bool(false)
  io.decex.memOp.store := Bool(false)
  io.decex.memOp.hword := Bool(false)
  io.decex.memOp.byte := Bool(false)
  io.decex.memOp.zext := Bool(false)
  io.decex.memOp.typ := ldtype
  io.decex.call := Bool(false)
  io.decex.ret := Bool(false)
  io.decex.brcf := Bool(false)

  io.decscex.spill := Bits(0)
  io.decscex.fill := Bits(0)
  io.decscex.free := Bits(0)
  io.decscex.nSpill := SInt(1)
  io.decscex.nFill := SInt(1)
  io.decscex.sp := UInt(0)  
  io.memscdec.mTop := UInt(0)  
  // Long immediates set this
  longImm := Bool(false)

  // Load/stores and stack control operations set this
  isMem := Bool(false)
  isStack := Bool(false)
  isSTC := Bool(false)
  stcVal := io.exdec.sp

  // Everything except calls uses the default
  dest := instr(21, 17)

  // ALU long immediate (Bit 31 is set as well)
  when(opcode === OPCODE_ALUL) {
	io.decex.aluOp(0).func := func
    io.decex.immOp(0) := Bool(true)
    longImm := Bool(true)
    io.decex.wrRd(0) := Bool(true)
  }
  // Stack control
  val reserveSize = io.memscdec.mTop - io.exdec.sp + stcImm - (UInt(SCACHE_SIZE) << UInt(2))
  val ensureSize = io.decscex.sp + stcImm - io.memscdec.mTop
  when(opcode === OPCODE_STC) {
    io.decscex.sp := io.exdec.sp
	switch(stcfun) {
	  is(STC_SRES) {
		io.decex.aluOp(0).isSTC := Bool(true)
		isSTC := Bool(true)
		io.decex.immOp(0) := Bool(true)
		stcVal := io.exdec.sp - stcImm
		io.decscex.nSpill := reserveSize(ADDR_WIDTH - 1, 2)
		io.decscex.spill := Mux(reserveSize > UInt(0), Bits(1), Bits(0))
	  }
	  is (STC_SENS) {
	    io.decscex.nFill := ensureSize(ADDR_WIDTH - 1, 2)
	    io.decscex.fill := Mux(ensureSize > UInt(0), Bits(1), Bits(0))
	  }
	  is(STC_SFREE) {
		io.decex.aluOp(0).isSTC := Bool(true)
		isSTC := Bool(true)
		io.decex.immOp(0) := Bool(true)
		stcVal := io.exdec.sp + stcImm
		io.decscex.free := Bits(1)
	  }
	}
  }
  // Control-flow operations
  when(opcode === OPCODE_CFL_CALL) {
    io.decex.immOp(0) := Bool(true)
    io.decex.call := Bool(true)
    io.decex.wrRd(0) := Bool(true)
	dest := Bits("b11111")
  }
  when(opcode === OPCODE_CFL_BR) {
    io.decex.immOp(0) := Bool(true)
	io.decex.jmpOp.branch := Bool(true)
  }
  when(opcode === OPCODE_CFL_BRCF) {
    io.decex.immOp(0) := Bool(true)
    io.decex.brcf := Bool(true)
  }
  when(opcode === OPCODE_CFL_CFLI) {
	switch(func) {
	  is(JFUNC_CALL) {
		io.decex.call := Bool(true)
		io.decex.wrRd(0) := Bool(true)
		dest := Bits("b11111")
	  }
	  is(JFUNC_BR) {
		io.decex.jmpOp.branch := Bool(true)
	  }
	  is(JFUNC_BRCF) {
		io.decex.brcf := Bool(true)
	  }
	}
  }
  when(opcode === OPCODE_CFL_RET) {
    io.decex.ret := Bool(true)
  }

  val shamt = UInt()
  shamt := UInt(0)
  // load
  when(opcode === OPCODE_LDT) {
    isMem := Bool(true)
    io.decex.memOp.load := Bool(true)
    io.decex.wrRd(0) := Bool(true)
    switch(ldsize) {
      is(MSIZE_W) {
        shamt := UInt(2)
      }
      is(MSIZE_H) {
        shamt := UInt(1)
        io.decex.memOp.hword := Bool(true)
      }
      is(MSIZE_B) {
        io.decex.memOp.byte := Bool(true)
      }
      is(MSIZE_HU) {
        shamt := UInt(1)
        io.decex.memOp.hword := Bool(true)
        io.decex.memOp.zext := Bool(true)
      }
      is(MSIZE_BU) {
        io.decex.memOp.byte := Bool(true)
        io.decex.memOp.zext := Bool(true)
      }
      // ignore split load for now
    }
	io.decex.memOp.typ := ldtype;
	when(ldtype === MTYPE_S) {
	  isStack := Bool(true)
	}
  }
  // store
  when(opcode === OPCODE_STT) {
    isMem := Bool(true)
    io.decex.memOp.store := Bool(true)
    switch(stsize) {
      is(MSIZE_W) {
        shamt := UInt(2)
      }
      is(MSIZE_H) {
        shamt := UInt(1)
        io.decex.memOp.hword := Bool(true)
      }
      is(MSIZE_B) {
        io.decex.memOp.byte := Bool(true)
      }
    }
	io.decex.memOp.typ := sttype;
	when(sttype === MTYPE_S) {
	  isStack := Bool(true)
	}
  }

  // Offset for loads/stores
  val addrImm = Bits()
  addrImm := Cat(Bits(0), instr(6, 0))
  switch(shamt) {
    is(UInt(1)) { addrImm := Cat(Bits(0), instr(6, 0), Bits(0, width = 1)) }
    is(UInt(2)) { addrImm := Cat(Bits(0), instr(6, 0), Bits(0, width = 2)) }
  }

  // Immediate value
  io.decex.immVal(0) := Mux(isSTC, stcVal,
							Mux(isStack, addrImm + io.exdec.sp,
								Mux(isMem, addrImm,
									Mux(longImm, decReg.instr_b,
										Cat(Bits(0), instr(11, 0))))))
  // we could mux the imm / register here as well
  
  // Immediate for absolute calls
  io.decex.callAddr := Cat(Bits(0), instr(21, 0), Bits("b00")).toUInt()

  // Immediate for branch is sign extended, not extended for call
  // PC-relative value is precomputed here
  io.decex.jmpOp.target := decReg.pc + Cat(Fill(PC_SIZE - 22, instr(21)), instr(21, 0))
  io.decex.jmpOp.reloc := decReg.reloc

  // PC-relative address for brcf
  // TODO: this goes away when we make brcf like calls
  io.decex.brcfAddr := Cat(io.decex.jmpOp.target + decReg.reloc, Bits("b00").toUInt)

  // Pass on PC
  io.decex.pc := decReg.pc

  // Set destination address
  io.decex.rdAddr(0) := dest

  // Disable register write on register 0
  for (i <- 0 until PIPE_COUNT) {
	when(io.decex.rdAddr(i) === Bits("b00000")) {
      io.decex.wrRd(i) := Bool(false)
	}
  }
}

