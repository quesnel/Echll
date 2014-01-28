#
# Simple ycm_extra_conf.py for vle-template
#

import os
import ycm_core

flags = [
'-Wall',
'-Wextra',
'-Werror',
'-std=c++11',
#'-fexceptions',
#'-DNDEBUG',
'-x',
'c++',
'-I',
'.',
'-I',
'./src/lib',
'-I',
'./lib',
'-I',
'./src/bin',
'-isystem',
'/usr/include',
]

def FlagsForFile( filename ):
  return {
    'flags': flags,
    'do_cache': True
  }
