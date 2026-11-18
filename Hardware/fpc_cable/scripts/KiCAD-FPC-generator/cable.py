from fpc_generator import *

cable_top = None
cable_bot = Cable(0.2, .8, .3, .3, .3, .3, .3, .8, 0.2)
sections = [
    Down(445),
    Curve(3),
    Right(25),
]
generate_cable("cable.pretty/fpc_cable.kicad_mod", cable_top, cable_bot, sections)
