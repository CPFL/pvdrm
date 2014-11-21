# -*- coding: utf-8 -*-
#  Copyright (C) 2014 Yusuke Suzuki <yusuke.suzuki@sslab.ics.keio.ac.jp>
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
#  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
#  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TABLE = {
    '3.6.11' => {
        'dir' => '/lib/modules/3.6.11-030611-generic/build',
        'files' => [
            [ 'http://kernel.ubuntu.com/~kernel-ppa/mainline/v3.6.11-raring/linux-headers-3.6.11-030611_3.6.11-030611.201212171335_all.deb', 'all.deb' ],
            [ 'http://kernel.ubuntu.com/~kernel-ppa/mainline/v3.6.11-raring/linux-headers-3.6.11-030611-generic_3.6.11-030611.201212171335_amd64.deb', 'amd64.deb' ],
        ],
    },
    '3.16.7' => {
        'dir' => '/lib/modules/3.16.7-031607-generic/build',
        'files' => [
            [ 'http://kernel.ubuntu.com/~kernel-ppa/mainline/v3.16.7-utopic/linux-headers-3.16.7-031607_3.16.7-031607.201410301735_all.deb', 'all.deb' ],
            [ 'http://kernel.ubuntu.com/~kernel-ppa/mainline/v3.16.7-utopic/linux-headers-3.16.7-031607-generic_3.16.7-031607.201410301735_amd64.deb', 'amd64.deb' ],
        ],
    },
    '3.17.3' => {
        'dir' => '/lib/modules/3.17.3-031703-generic/build',
        'files' => [
            [ 'http://kernel.ubuntu.com/~kernel-ppa/mainline/v3.17.3-vivid/linux-headers-3.17.3-031703_3.17.3-031703.201411141335_all.deb', 'all.deb'],
            [ 'http://kernel.ubuntu.com/~kernel-ppa/mainline/v3.17.3-vivid/linux-headers-3.17.3-031703-generic_3.17.3-031703.201411141335_amd64.deb', 'amd64.deb'],
        ],
    },
}

def main
    version = ARGV.first
    data = TABLE[version]
    data['files'].each {|pair|
        puts "downloading #{pair.first}"
        system "wget '#{pair.first}' -O '#{pair.last}'"
        puts "installing #{pair.first}"
        system "sudo dpkg -i '#{pair.last}'"
    }
    system "echo '#{data['dir']}' > KDIR"
end

main

# vim: set sw=4 ts=4 et tw=80 :
