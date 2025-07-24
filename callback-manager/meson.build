project(
    'callback-manager',
    'cpp',
    version: '1.1.1',
    meson_version: '>=1.1.1',
    default_options: ['cpp_std=c++23'],
)

# Compiler flags
cpp_args = [
    '-Werror',
    '-Wall',
    '-Wextra',
    '-Wshadow',
    '-Wnon-virtual-dtor',
    '-Wold-style-cast',
    '-Wcast-align',
    '-Wunused',
    '-Woverloaded-virtual',
    '-Wpedantic',
    '-Wconversion',
    '-Wmisleading-indentation',
    '-Wduplicated-cond',
    '-Wduplicated-branches',
    '-Wlogical-op',
    '-Wnull-dereference',
    '-Wuseless-cast',
    '-Wdouble-promotion',
    '-Wformat=2',
    '-fno-rtti',
]

# Boost library
boost_dep = dependency('boost', required: true)

# Definitions
add_project_arguments(
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ASIO_DISABLE_THREADS',
    language: 'cpp',
)

# Include directories
inc = include_directories('include')

# Executable
#executable('callback-manager', 'src/callback_manager.cpp',
#  include_directories : inc,
#  cpp_args : cpp_args,
#  dependencies : [boost_dep],
#  link_with : ['systemd', 'i2c', 'sdbusplus', 'stdc++fs', 'gpiodcxx']
#)

boost = dependency('boost', version: '1.73.0', required: false)
sdbusplus = dependency('sdbusplus', required: false)



systemd_dep = dependency('systemd', required: true)
incdir = include_directories('include')
executable(
    'callback-manager',
    'src/callback_manager.cpp',
    include_directories: incdir,
    dependencies: [boost, sdbusplus, systemd_dep],
)
# Systemd service files
systemd_system_unit_dir = systemd_dep.get_pkgconfig_variable(
    'systemdsystemunitdir',
)

install_data(
    'service_files/callback-manager.service',
    install_dir: systemd_system_unit_dir,
)
