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
 * Definitions for OCP ports for split cache
 * 
 * Authors: Wolfgang Puffitsch (wpuffitsch@gmail.com)
 * 
 */

package ocp

import Chisel._
import Node._

object OcpCache {
  val STACK_CACHE = Bits("b00")
  val DATA_CACHE  = Bits("b10")
  val UNCACHED    = Bits("b11")
}

// Cache masters provide address space signal
class OcpCacheMasterSignals(addrWidth : Int, dataWidth : Int)
  extends OcpCoreMasterSignals(addrWidth, dataWidth) {
  val AddrSpace = Bits(width = 2)

  // This does not really clone, but Data.clone doesn't either
  override def clone() = {
    val res = new OcpCacheMasterSignals(addrWidth, dataWidth)
  	res.asInstanceOf[this.type]
  }

  override def reset() = {
	super.reset()
	AddrSpace := OcpCache.UNCACHED
  }
}

// Reset values for master signals
object OcpCacheMasterSignals {
  def resetVal[T <: OcpCacheMasterSignals](sig : T) : T = {
	val res = sig.clone
	res.reset()
	res
  }
  def resetVal(addrWidth : Int, dataWidth : Int) : OcpCacheMasterSignals = {
	resetVal(new OcpCacheMasterSignals(addrWidth, dataWidth))
  }
}

// Master port
class OcpCacheMasterPort(addrWidth : Int, dataWidth : Int) extends Bundle() {
  // Clk is implicit in Chisel
  val M = new OcpCacheMasterSignals(addrWidth, dataWidth).asOutput
  val S = new OcpSlaveSignals(dataWidth).asInput 
}

// Slave port is reverse of master port
class OcpCacheSlavePort(addrWidth : Int, dataWidth : Int) extends Bundle() {
  // Clk is implicit in Chisel
  val M = new OcpCacheMasterSignals(addrWidth, dataWidth).asInput
  val S = new OcpSlaveSignals(dataWidth).asOutput
}

// Provide a "bus" with a master port and a slave port to simplify plumbing
class OcpCacheBus(addrWidth : Int, dataWidth : Int) extends Module {
  val io = new Bundle {
    val slave = new OcpCacheSlavePort(addrWidth, dataWidth)
    val master = new OcpCacheMasterPort(addrWidth, dataWidth)
  }
  io.master <> io.slave
}
