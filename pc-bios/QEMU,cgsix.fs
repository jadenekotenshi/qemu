\
\ Fcode payload for QEMU CG6 graphics card
\
\ Adapted from OpenBIOS' cgthree.fs (C) Copyright 2013 Mark Cave-Ayland.
\ Unlike the cg3 one this reads the resolution from the FHC register, as the
\ firmware of a real cg6 does, so QEMU only has to set the FHC.
\

fcode-version3

\
\ Registers
\

h# 200000 constant cg6-off-bt
h# 1000 constant /cg6-off-bt
h# 300000 constant cg6-off-fhc
h# 1000 constant /cg6-off-fhc
h# 800000 constant cg6-off-fb

: >cg6-reg-spec ( offset size -- encoded-reg )
  >r 0 my-address d+ my-space encode-phys r> encode-int encode+
;

: cg6-reg
  \ the whole slot, like the cg3 and like the real card's single entry
  h# 0 h# 1000000 >cg6-reg-spec
  " reg" property
;

: do-map-in ( offset size -- virt )
  >r my-space r> " map-in" $call-parent
;

: do-map-out ( virt size )
  " map-out" $call-parent
;

-1 value cg6-bt
-1 value fhc-addr
-1 value fb-addr
0 value cg6-w
0 value cg6-h

\
\ DAC (a Brooktree Bt458: address at +0, colour map data at +4)
\

: dac! ( data reg# -- )
  cg6-bt + c!
;

external

: color!  ( r g b c# -- )
  0 dac!       ( r g b )
  swap rot     ( b g r )
  4 dac!       ( b g )
  4 dac!       ( b )
  4 dac!       (  )
;

headerless

\
\ Geometry: FHC bits 12:11 select 1024x768, 1152x900, 1280x1024 or 1600x1200
\

: fhc-map
  cg6-off-fhc /cg6-off-fhc do-map-in to fhc-addr
;

: fhc-unmap
  fhc-addr /cg6-off-fhc do-map-out
  -1 to fhc-addr
;

: fhc-res ( -- n )
  fhc-addr l@ h# 1800 and h# b rshift
;

: set-geometry ( -- )
  fhc-res case
    0 of h# 400 h# 300 endof
    1 of h# 480 h# 384 endof
    2 of h# 500 h# 400 endof
    3 of h# 640 h# 4b0 endof
  endcase
  to cg6-h to cg6-w
;

\ the video RAM size in megabytes: QEMU patches this constant when it loads
\ the PROM, as a real PROM would size the RAM by probing it
: vm-size ( -- megabytes )
  h# 5a5a5a5a
;

\
\ Mapping
\

: dac-map
  cg6-off-bt /cg6-off-bt do-map-in to cg6-bt
;

: fb-map
  cg6-off-fb vm-size h# 100000 * do-map-in to fb-addr
;

: map-regs
  dac-map fb-map
;

\
\ Installation
\

" cgsix" device-name
" display" device-type
" SUNW,501-2325" model

: qemu-cg6-driver-install ( -- )
  cg6-bt -1 = if
    map-regs

    \ Initial pallette taken from Sun's "Writing FCode Programs"
    h# ff h# ff h# ff h# 0  color!    \ Background white
    h# 0  h# 0  h# 0  h# ff color!    \ Foreground black
    h# 64 h# 41 h# b4 h# 1  color!    \ SUN-blue logo

    fb-addr to frame-buffer-adr
    default-font set-font

    frame-buffer-adr encode-int " address" property

    cg6-w cg6-h over char-width / over char-height /
    fb8-install
  then
;

: qemu-cg6-driver-init

  cg6-reg

  fhc-map
  set-geometry
  fhc-unmap

  cg6-h encode-int " height" property
  cg6-w encode-int " width" property
  cg6-w encode-int " linebytes" property
  cg6-w encode-int " awidth" property
  h# 8 encode-int " depth" property
  vm-size encode-int " vmsize" property

  h# 39 encode-int 0 encode-int encode+ " intr" property

  " SUNW" encode-string " manufacturer" property
  " ISO8859-1" encode-string " character-set" property
  h# c encode-int " cursorshift" property

  ['] qemu-cg6-driver-install is-install
;

qemu-cg6-driver-init

end0
