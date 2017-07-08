/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "container_utils.h"
#include <rtt/Logger.hpp>

uint32_t randMinMax(uint32_t min, uint32_t max) {
    return rand()%(max - min + 1) + min;
}

uint32_t calculateChecksum(common_interfaces_test_msgs::Container& cont_in) {
    return
        cont_in.cont1.sub1.field1 +
        cont_in.cont1.sub1.field2 +
        cont_in.cont1.sub2.field1 +
        cont_in.cont1.sub2.field2 +
        cont_in.cont1.field1 +
        cont_in.cont1.field2 +
        cont_in.cont2.sub1.field1 +
        cont_in.cont2.sub1.field2 +
        cont_in.cont2.sub2.field1 +
        cont_in.cont2.sub2.field2 +
        cont_in.cont2.field1 +
        cont_in.cont2.field2 +
        cont_in.cont3.sub1.field1 +
        cont_in.cont3.sub1.field2 +
        cont_in.cont3.sub2.field1 +
        cont_in.cont3.sub2.field2 +
        cont_in.cont3.field1 +
        cont_in.cont3.field2 +
        cont_in.cont4.sub1.field1 +
        cont_in.cont4.sub1.field2 +
        cont_in.cont4.sub2.field1 +
        cont_in.cont4.sub2.field2 +
        cont_in.cont4.field1 +
        cont_in.cont4.field2 +
        cont_in.field1;
}

void randomContainerData(common_interfaces_test_msgs::Container& cont_in) {
  cont_in.cont1.sub1.field1 = randMinMax(1, 100);
  cont_in.cont1.sub1.field1_valid = true;
  cont_in.cont1.sub1.field2 = randMinMax(1, 100);
  cont_in.cont1.sub1_valid = true;
  cont_in.cont1.sub2.field1 = randMinMax(1, 100);
  cont_in.cont1.sub2.field1_valid = true;
  cont_in.cont1.sub2.field2 = randMinMax(1, 100);
  cont_in.cont1.field1 = randMinMax(1, 100);
  cont_in.cont1.field1_valid = true;
  cont_in.cont1.field2 = randMinMax(1, 100);
  cont_in.cont1_valid = true;
  cont_in.cont2.sub1.field1 = randMinMax(1, 100);
  cont_in.cont2.sub1.field2 = randMinMax(1, 100);
  cont_in.cont2.sub2.field1 = randMinMax(1, 100);
  cont_in.cont2.sub2.field2 = randMinMax(1, 100);
  cont_in.cont2.field1 = randMinMax(1, 100);
  cont_in.cont2.field1_valid = true;
  cont_in.cont2.field2 = randMinMax(1, 100);
  cont_in.cont3.sub1.field1 = randMinMax(1, 100);
  cont_in.cont3.sub1.field2 = randMinMax(1, 100);
  cont_in.cont3.sub2.field1 = randMinMax(1, 100);
  cont_in.cont3.sub2.field2 = randMinMax(1, 100);
  cont_in.cont3.field1 = randMinMax(1, 100);
  cont_in.cont3.field2 = randMinMax(1, 100);
  cont_in.cont3_valid = true;
  cont_in.cont4.sub1.field1 = randMinMax(1, 100);
  cont_in.cont4.sub1.field2 = randMinMax(1, 100);
  cont_in.cont4.sub1_valid = true;
  cont_in.cont4.sub2.field1 = randMinMax(1, 100);
  cont_in.cont4.sub2.field2 = randMinMax(1, 100);
  cont_in.cont4.field1 = randMinMax(1, 100);
  cont_in.cont4.field2 = randMinMax(1, 100);
  cont_in.field1 = randMinMax(1, 100);
  cont_in.field1_valid = true;

  // checksum
  cont_in.field2 = calculateChecksum(cont_in);
}

