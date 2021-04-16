project('thermocam-pcb', ['cpp'],
	default_options : [ 'cpp_std=c++11' ]
       )

cxx = meson.get_compiler('cpp')

# Get rid of some warnings from WIC SDK and Crow
add_global_arguments('-Wno-unknown-pragmas', '-Wno-non-virtual-dtor', language : 'cpp')

if cxx.has_header('CameraCenter.h', args : '-I@0@/include'.format(get_option('wic_home')))
  wic_dep = declare_dependency(
    include_directories : include_directories(get_option('wic_home') / 'include'),
    dependencies : [
      cxx.find_library('WIC_SDK', dirs : '@0@/lib'.format(get_option('wic_home'))),
      cxx.find_library('WT-lib', dirs : '@0@/lib'.format(get_option('wic_home'))),
    ]
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

opencv_dep = dependency('opencv4', required : true)

version_h = vcs_tag(input : 'version.h.in', output : 'version.h',
		    command : get_option('version') != ''
		              ? ['echo', get_option('version')]
                              : ['git', 'describe', '--always'])

webserver = static_library('webserver', ['webserver.cpp'],
        dependencies: [
          opencv_dep,
	  ebus_dep,
          dependency('boost', modules : ['system']),
          dependency('threads')
        ],
       )

executable('thermocam-pcb',
	   [
	     'thermocam-pcb.cpp',
	     'point-tracking.cpp',
	     'img_stream.cpp',
	     'thermo_img.cpp',
	   ],
	   dependencies: [
	     opencv_dep,
	     wic_dep,
	     ebus_dep,
	     dependency('libsystemd'),
	   ],
	   link_with : webserver,
	   install : true,
	  )

executable('track-test', ['support/track-test.cpp'],
     dependencies: [
       opencv_dep,
       dependency('openmp')
     ],
    )

conf_data = configuration_data()
conf_data.set('EBUS_HOME', get_option('ebus_home'))
conf_data.set('WIC_HOME', get_option('wic_home'))
conf_data.set('OPENCV_HOME', opencv_dep.get_pkgconfig_variable('prefix'))
conf_data.set('PREFIX', get_option('prefix'))

configure_file(input : 'run.in',
               output : 'run',
               configuration : conf_data,
	       install_dir : 'bin'
	      )

configure_file(input : 'thermocam-pcb.service.in',
               output : 'thermocam-pcb.service',
               configuration : conf_data,
	       install_dir : 'lib/systemd'
	      )
