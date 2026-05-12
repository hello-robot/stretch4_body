project('hello-robot-stretch4-body', 'cpp',
  version: run_command('python3', 'get_version.py', check: true).stdout().strip(),
  default_options: ['cpp_std=c++17', 'b_ndebug=if-release'])

fs = import('fs')
if not fs.exists('stretch4_body/core/transport/cpp/readerwriterqueue/readerwriterqueue.h')
    message('readerwriterqueue submodule is missing, attempting to clone from git...')
    r = run_command('git', 'submodule', 'update', '--init', check: false, env: {'GIT_TERMINAL_PROMPT': '0'})
    if r.returncode() != 0
        warning('Failed to update git submodules: ' + r.stderr())
    endif
endif

py = import('python').find_installation()

# Define paths
src_dir = 'stretch4_body'

# Recursively install Python package
install_subdir(src_dir,
  install_dir: py.get_install_dir()
)

# SCServo Library
scservo_dir = src_dir / 'core' / 'feetech' / 'SCSerial'
scservo_src = [
    scservo_dir / 'SCS.cpp',
    scservo_dir / 'SCSCL.cpp',
    scservo_dir / 'SCSerial.cpp',
    scservo_dir / 'SMSBL.cpp',
    scservo_dir / 'SMSCL.cpp',
    scservo_dir / 'SMS_STS.cpp'
]
scservo_lib = shared_library('SCServo',
    scservo_src,
    install: true,
    install_dir: py.get_install_dir() / src_dir / 'core' / 'transport'
)

# Transport Library
transport_dir = src_dir / 'core' / 'transport' / 'cpp'
transport_src = [
    transport_dir / 'transport.cpp',
    transport_dir / 'transport_framing.cpp',
    transport_dir / 'crc16.cpp'
]

transport_lib = shared_library('transport',
    transport_src,
    include_directories: scservo_dir,
    link_with: scservo_lib,
    install: true,
    install_dir: py.get_install_dir() / src_dir / 'core' / 'transport'
)

# Scripts
glob_script = '''
import glob
files = glob.glob('tools/*.sh') + glob.glob('tools/factory/*.sh')
print(';'.join(files))
'''
r = run_command(py, '-c', glob_script, check: true)
if r.stdout().strip() != ''
    scripts = r.stdout().strip().split(';')
    foreach script : scripts
        if script != ''
            install_data(script, install_dir: get_option('bindir'))
        endif
    endforeach
endif
