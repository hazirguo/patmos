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
 * IO component of Patmos.
 *
 * Authors: Martin Schoeberl (martin@jopdesign.com)
 *          Wolfgang Puffitsch (wpuffitsch@gmail.com)
 *
 */

package patmos

import Chisel._
import Node._

import Constants._

import ocp._
import util._
import io.CoreDevice

class InOut() extends Module {
  val io = Config.getInOutIO()

  // Compute selects
  val selIO = io.memInOut.M.Addr(ADDR_WIDTH-1, ADDR_WIDTH-4) === Bits("b1111")
  val selNI = io.memInOut.M.Addr(ADDR_WIDTH-1, ADDR_WIDTH-4) === Bits("b1110")

  val selISpm = !selIO & !selNI & io.memInOut.M.Addr(ISPM_ONE_BIT) === Bits(0x1)
  val selSpm = !selIO & !selNI & io.memInOut.M.Addr(ISPM_ONE_BIT) === Bits(0x0)

  val selComConf = selNI & io.memInOut.M.Addr(ADDR_WIDTH-5) === Bits("b0")
  val selComSpm  = selNI & io.memInOut.M.Addr(ADDR_WIDTH-5) === Bits("b1")

  val MAX_IO_DEVICES = 0x10

  val selDeviceVec = Vec.fill(MAX_IO_DEVICES) { Bool() }
  val deviceSVec = Vec.fill(MAX_IO_DEVICES) { OcpSlaveSignals.resetVal(DATA_WIDTH) }
  for (i <- 0 to MAX_IO_DEVICES-1) {
    selDeviceVec(i) := selIO & io.memInOut.M.Addr(11, 8) === Bits(i)
    deviceSVec(i) := OcpSlaveSignals.resetVal(DATA_WIDTH)
  }

  // Register selects
  val selSpmReg = Reg(init = Bool(false))
  val selComConfReg = Reg(init = Bool(false))
  val selComSpmReg = Reg(init = Bool(false))

  val selDeviceReg = Vec.fill(MAX_IO_DEVICES) { Reg(Bool()) }

  when(io.memInOut.M.Cmd != OcpCmd.IDLE) {
	selSpmReg := selSpm
	selComConfReg := selComConf
	selComSpmReg := selComSpm

	selDeviceReg := selDeviceVec
  }

  // Dummy ISPM (create fake response)
  val ispmCmdReg = Reg(next = Mux(selISpm, io.memInOut.M.Cmd, OcpCmd.IDLE))
  val ispmResp = Mux(ispmCmdReg === OcpCmd.IDLE, OcpResp.NULL, OcpResp.DVA)

  // The SPM
  val spm = Module(new Spm(DSPM_SIZE))
  spm.io.M := io.memInOut.M
  spm.io.M.Cmd := Mux(selSpm, io.memInOut.M.Cmd, OcpCmd.IDLE)
  val spmS = spm.io.S

  // The communication configuration, including bridge to OcpIO interface
  val comConf = Module(new OcpCoreBus(ADDR_WIDTH, DATA_WIDTH))
  comConf.io.slave.M := io.memInOut.M
  comConf.io.slave.M.Cmd := Mux(selComConf, io.memInOut.M.Cmd, OcpCmd.IDLE)
  val comConfS = comConf.io.slave.S
  val comConfIO = Module(new OcpIOBus(ADDR_WIDTH, DATA_WIDTH))
  io.comConf.M := comConfIO.io.master.M
  comConfIO.io.master.S := io.comConf.S
  val comConfBridge = new OcpIOBridge(comConf.io.master, comConfIO.io.slave)

  // The communication scratchpad
  io.comSpm.M := io.memInOut.M
  io.comSpm.M.Cmd := Mux(selComSpm, io.memInOut.M.Cmd, OcpCmd.IDLE)
  val comSpmS = io.comSpm.S

  for (devConf <- Config.conf.Devs) {
    val dev = Config.createDevice(devConf).asInstanceOf[CoreDevice]
    // connect ports
    dev.io.ocp.M := io.memInOut.M
    dev.io.ocp.M.Cmd := Mux(selDeviceVec(devConf.offset), io.memInOut.M.Cmd, OcpCmd.IDLE)
    deviceSVec(devConf.offset) := dev.io.ocp.S
    Config.connectIOPins(devConf.name, io, dev.io)
  }

  // Return data to pipeline
  io.memInOut.S.Data := spmS.Data
  when(selComConfReg) { io.memInOut.S.Data := comConfS.Data }
  when(selComSpmReg)  { io.memInOut.S.Data := comSpmS.Data }
  for (i <- 0 to MAX_IO_DEVICES-1) {
    when(selDeviceReg(i)) { io.memInOut.S.Data := deviceSVec(i).Data }
  }

  // Merge responses
  io.memInOut.S.Resp := ispmResp | spmS.Resp |
                        comConfS.Resp | comSpmS.Resp |
                        deviceSVec.map(_.Resp).fold(OcpResp.NULL)(_|_)
}
