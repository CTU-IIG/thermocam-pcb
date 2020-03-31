project('thermocam-pcb', ['cpp'],
	default_options : [ 'cpp_std=c++11' ]
       )

cxx = meson.get_compiler('cpp')

if cxx.has_header('CameraCenter.h', args : '-I@0@/include'.format(get_option('wic_home')))
  wic_dep = declare_dependency(
    include_directories : include_directories(get_option('wic_home') / 'include'),
    dependencies : [ cxx.find_library('WIC_SDK', dirs : '@0@/lib'.format(get_option('wic_home'))) ]
  )
else
  error('WIC SDK not found in @0@. Run meson -Dwic_home='.format(get_option('wic_home')))
endif

if cxx.has_header('EbVersion.h', args : '-I@0@/include'.format(get_option('ebus_home')))
  libs = []
  foreach lib : 'PvBase PvDevice PvBuffer PvGenICam PvTransmitter PvVirtualDevice PvAppUtils PvPersistence PvSerial PvStream'.split()
    libs += cxx.find_library(lib, dirs : '@0@/lib'.format(get_option('ebus_home')))
  endforeach
  ebus_dep = declare_dependency(
    include_directories : include_directories(get_option('ebus_home') / 'include'),
    compile_args : [ '-D_UNIX_', '-D_LINUX_' ],
    dependencies : libs,
    link_args : [
      '-Wl,-rpath=@0@/lib/genicam/bin/Linux64_x64'.format(get_option('ebus_home')),
    ]
  )
else
  error('eBUS SDK not found in @0@. Run meson -Debus_home='.format(get_option('ebus_home')))
endif

opencv_dep = dependency('opencv', required : false)
if not opencv_dep.found()
  opencv_dep = dependency('opencv4', required : true)
endif

version_h = vcs_tag(input : 'version.h.in', output : 'version.h',
		    command : ['git', 'describe', '--always'])

cpp_args = []
if cxx.has_header('opencv2/videoio/legacy/constants_c.h', dependencies : opencv_dep)
  cpp_args += '-DHAVE_LEGACY_VIDEOIO=1'
endif

executable('thermocam-pcb', ['thermocam-pcb.cpp'],
	   dependencies: [
	     opencv_dep,
	     wic_dep,
	     ebus_dep,
	     dependency('boost', modules : ['system']),
	     dependency('threads'),
	   ],
	   cpp_args : cpp_args,
	  )
