project(
    'hsbp-manager',
    'cpp',
    version: '1.1.1',
    meson_version: '>=1.1.1',
    default_options: ['cpp_std=c++23'],
)

# Compiler flags
cpp_args = [
    '-lstdc++fs',
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

cpp = meson.get_compiler('cpp')
boost = dependency('boost', version: '1.86.0', required: false)
sdbusplus = dependency('sdbusplus', required: true)
i2c_dep = cpp.find_library('i2c')
gpiodcxx = dependency('libgpiodcxx', default_options: ['bindings=cxx'])

incdir = include_directories('include')

executable(
    'hsbp-manager',
    'src/hsbp_manager.cpp',
    include_directories: incdir,
    dependencies: [boost, i2c_dep, sdbusplus, gpiodcxx],
)
# Systemd service files
systemd_system_unit_dir = dependency('systemd').get_variable(
    'systemdsystemunitdir',
)

install_data(
    'service_files/hsbp-manager.service',
    install_dir: systemd_system_unit_dir,
)
