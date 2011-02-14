( Main for WGE firmware                      JCB 13:24 08/24/10)

warnings off
require tags.fs

include crossj1.fs
meta
    : TARGET? 1 ;
    : ONBENCH 0 ;
    : DO-XORLINE 0 ;
    : build-debug? 1 ;
    : build-tcp? 0 ;

include basewords.fs
target
include hwwge.fs
include boot.fs
include doc.fs

4 org
module[ eveything"
include nuc.fs

create mac         6 allot
create serial      4 allot
create camera_name 40 allot

: net-my-mac mac dup @ swap 2+ dup @ swap 2+ @ ;

: halt  [char] # emit begin again ;

: alarm
    s" ** failed selftest: code " type hex2 cr
    halt
;

include version.fs
include parseversion.fs

: rapidflash
    time 2+ @ led ! ;
: morseflash ( code -- )
    time 2+ @ 2/ h# f and rshift invert led ! ;

include morse.fs
include time.fs
include mac.fs
include continuation.fs
include packet.fs
include ip0.fs
include defines_tcpip.fs
include defines_tcpip2.fs
include arp.fs
include ip.fs
include dhcp.fs

include spi.fs
include flash.fs

: hardreset 
    true trig_reconf ! begin again ;

: softreset
    ['] emit-uart 'emit !
    sleep1 [char] a emit cr sleep1
    begin dsp h# ff and while drop repeat
    begin dsp d# 8 rshift while r> drop repeat
    dsp hex4 cr
    d# 0 >r
    sleep1 [char] b emit cr sleep1
;

include mt9v.fs

: setrouter ( router subnet -- )
    ip-subnetmask 2! ip-router 2!
    arp-reset
    net-my-ip arp-lookup drop arp-announce ;

: guess-mask ( ip -- )
    ip# 0.0.0.0 \ the world is my LAN
    ip# 0.0.0.0
    setrouter

    ip# 255.255.0.0 dand
    2dup ip# 10.0.0.0 d= if
        ip# 10.0.0.1 
        ip# 255.255.248.0 
        setrouter
    then
    2dup ip# 10.68.0.0 d= if
        ip# 10.68.0.1 
        ip# 255.255.255.0 
        setrouter
    then
    ip# 10.69.0.0 d= if
        ip# 10.69.0.11
        ip# 255.255.255.0 
        setrouter
    then
;

: ip-addr! ( ip -- )
    2dup ip-addr 2@ d<> if
        2dup ip-addr 2!
        guess-mask
        arp-reset
    else
        2drop
    then
;

include wge.fs
build-tcp? [IF]
    include newtcp.fs
    include html.fs
    include http.fs
    include tcpservice.fs
[THEN]


( IP address formatting                      JCB 14:50 10/26/10)

: #ip1  h# ff and s>d #s 2drop ;
: #.    [char] . hold ;
: #ip2  dup #ip1 #. d# 8 rshift #ip1 ;
: #ip   ( ip -- c-addr u) dup #ip2 #. over #ip2 ;

( net-watchdog                               JCB 09:26 10/13/10)

2variable net-watchdog
: net-watch-reset
    d# 10000000. net-watchdog setalarm ;
: net-watch-expired?
    net-watchdog isalarm ;

: preip-handler
    begin
        enc-fullness
    while
        OFFSET_ETH_TYPE packet@ h# 800 =
        if
           dhcp-wait-offer
        then
        camera-handler
    repeat
;

: strlen ( addr -- u ) dup begin count 0= until swap - 1- ;

include epa.fs
: ntp-server 
   h# 02830a00.
;

: ntp-request
    d# 123 d# 9999
    ntp-server
    net-my-ip
    2over arp-lookup
    ( dst-port src-port dst-ip src-ip *ethaddr )
    udp-header
    h# 2304 enc-pkt-, h# 04ec enc-pkt-, 
    d# 6 enc-pkt-,0

    d# 4 enc-pkt-,0 \ originate
    d# 4 enc-pkt-,0 \ reference
    d# 4 enc-pkt-,0 \ receive
    \ d# 4 enc-pkt-,0 \ transmit
    time@ enc-pkt-d, d# 2 enc-pkt-,0
    udp-wrapup enc-send
;

: haveip-handler
    begin
        enc-fullness
    while
        net-watch-reset
        arp-handler
        OFFSET_ETH_TYPE packet@ h# 800 =
        if
            d# 2 OFFSET_IP_DSTIP enc-offset enc@n net-my-ip d=
            if
                icmp-handler
                \ IP_PROTO_TCP ip-isproto if servetcp then
                IP_PROTO_UDP ip-isproto
                ETH.IP.UDP.SOURCEPORT packet@ d# 123 = and
                ETH.IP.UDP.DESTPORT packet@ d# 9999 = and
                if
                    d# 2 ETH.IP.DSTIP enc-offset enc@n ip-pretty cr
                    time@ ETH.IP.UDP.NTP.ORIGINATE packetd@ d-
                    s" ntp delay " type
                    d. cr
                then
            then
            camera-handler
        then
        depth if .s cr then
        depth d# 6 u> if hardreset then
    repeat
;

: bench   
    cbench
    d# 1000 >r \ iterations
    time@
    r@ negate begin
        \ d# 33. d# 101. 2d+ drop drop
        \ d# 33 d# 101 +1c drop
        \ d# 23 s>q d# 11 d# 17 qm*/ qdrop
        progress

        d# 1 + dup d# 0 =
    until drop
    time@
    decimal s" bench: " type
    2swap d- d# 6800 r> m*/ d# 600. d- <# # # [char] . hold #s #> type
    s"  cycles" type cr
;

ONBENCH [IF]
    : banner
        cr cr
        d# 64 0do [char] * emit loop cr
        s" J1 running" type cr
        cr

        s" Imager:  " type imagerversion @ hex4 cr
        s" PCB rev: " type pcb_rev @ . cr
        s" HDL rev: " type hdl_version @ hex4 cr
        s" FW rev:  " type version type
            s"  reports as " type version version-n hex d. decimal cr
        s" serial:  " type serial 2@ d. cr
        s" MAC:     " type net-my-mac mac-pretty cr
        cr
    ;
    : phy-report s" PHY status: " type d# 1 mac-mii@ hex4 cr ;

    create prev d# 4 allot
    : clocker
        time@ prev 2@ d- d# 1000000. d> if
            time@ prev 2!
            time@ hex8 space
            cr

            \ ntp-server arp-lookup if ntp-request then
        then
    ;
[ELSE]
    : phy-report ;
[THEN]

include syslog.fs

: get-dhcp
    net-my-mac xor mt9v-random d+ dhcp-xid!
    d# 0. dhcp-alarm setalarm

    d# 1000
    begin
        net-my-ip d0=
    while
        dhcp-alarm isalarm if
            dhcp-discover
            2* d# 8000 min
            dup d# 1000 m* dhcp-alarm setalarm
        then
        preip-handler
    repeat
    snap
    drop
    depth if begin again then
;

: main
    decimal
    ['] emit-uart 'emit !
    mt9v-cold
    atmel-cold
    atmel-cfg-rd
    atmel-id-rd

    ONBENCH [IF]
        banner
    [THEN]

    net-my-mac mac-cold
    phy-report

    net-my-ip d0= if
        get-dhcp
    else
        net-my-ip guess-mask
    then

    arp-reset

    build-tcp? [IF]
        tcp-cold
    [THEN]

    ONBENCH [IF]
        dhcp-status
    [ELSE]
        begin
            haveip-handler syslog-server arp-lookup 0=
        while
            sleep.1
        repeat
        s" syslog -> " type syslog-server ip-pretty cr 
        syslog-cold ['] emit-syslog 'emit !
    [THEN]

    build-debug? [IF]
        s" booted serial://" type serial 2@ d.
        s" from " type
        h# 3ffe @ if s" mcs" else s" flash" then type
        s"  ip " type
        net-my-ip <# #ip #> type space
        s" xorline=" type
        XORLINE hex1 space
        version type
        cr
        s" ready" type cr
    [THEN]

    \ net-my-ip arp-lookup drop arp-announce

    net-watch-reset
    begin
        \ clocker
        inframe invert if
            haveip-handler
        then
        mt9v-cycle
        net-watch-expired? if
            net-watch-reset
            mt9v-cold
            net-my-mac mac-cold
        then
    again

    halt
;
]module

0 org

code 0jump
    \ h# 3e00 ubranch
    main ubranch
    main ubranch
end-code

meta

hex


: create-output-file w/o create-file throw to outfile-id ;
s" j1.mem" create-output-file
:noname
    ." @ 20000" cr
    4000 0 do i t@ s>d <# # # # # #> type cr 2 +loop
; execute

s" j1.bin" create-output-file
:noname 4000 0 do i t@ dup 8 rshift emit emit 2 +loop ; execute

s" lst" create-output-file
d# 0
h# 2000 disassemble-block

bye
