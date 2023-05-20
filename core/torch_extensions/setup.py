import glob
import os
import os.path as osp
import platform
import sys
from itertools import product

import torch
from setuptools import find_packages, setup
from torch.__config__ import parallel_info
from torch.utils.cpp_extension import (CUDA_HOME, BuildExtension, CppExtension,
                                       CUDAExtension)

__version__ = '0.0.1'

WITH_CUDA = False
if torch.cuda.is_available():
    WITH_CUDA = CUDA_HOME is not None or torch.version.hip

def get_extensions():
    extensions = []

    extensions_dir = osp.join('./')
    main_files = glob.glob(osp.join(extensions_dir, '*.cpp'))
    extensions_dir1 = osp.join('../src')
    main_files += glob.glob(osp.join(extensions_dir1, '*'))
    # remove generated 'hip' files, in case of rebuilds
    main_files = [path for path in main_files if 'hip' not in path]

    for main in main_files:
        define_macros = [('WITH_PYTHON', None)]
        undef_macros = []

        extra_compile_args = {'cxx': ['-O2']}

        info = parallel_info()
        if ('backend: OpenMP' in info and 'OpenMP not found' not in info
                and sys.platform != 'darwin'):
            extra_compile_args['cxx'] += ['-DAT_PARALLEL_OPENMP']
            extra_compile_args['cxx'] += ['-fopenmp']
        else:
            print('Compiling without OpenMP...')

        extra_compile_args['cxx'] += ['-fvisibility=hidden', '-std=c++17']

        extra_link_args = ['-s', '-lm', '-lstdc++fs', '-lgmp', '-lmpfr', '-lyaml-cpp', '-lboost_iostreams', '-lboost_system', '-lboost_filesystem']
        define_macros += [('WITH_CUDA', None)]
        nvcc_flags = os.getenv('NVCC_FLAGS', '')
        nvcc_flags = [] if nvcc_flags == '' else nvcc_flags.split(' ')
        nvcc_flags += ['-O2']
        extra_compile_args['nvcc'] = nvcc_flags
        nvcc_flags += ['--expt-relaxed-constexpr']

        sources = [main]

        extension = CUDAExtension("pyCoverageControl",
            sources,
            include_dirs=[extensions_dir],
            define_macros=define_macros,
            undef_macros=undef_macros,
            extra_compile_args=extra_compile_args,
            extra_link_args=extra_link_args,
        )
        extensions += [extension]

    return extensions

include_package_data = True

setup(
    name='pyCoverageControl',
    version=__version__,
    description=('pyCoverageControl: A Python library for Coverage Control'),
    author='Saurav Agarwal',
    author_email='SauravAg@upenn.edu',
    keywords=[
        'pytorch',
        'coverage-control',
        'graph-neural-networks',
    ],
    python_requires='>=3.10',
    ext_modules=get_extensions(),
    include_dirs=['/root/CoverageControl_ws/install/include', '/usr/local/include/eigen3'],
    cmdclass={
        'build_ext':
        BuildExtension
    },
    packages=find_packages(),
    include_package_data=include_package_data,
)
# setup(
#     name='pyCoverageControl',
#     version=__version__,
#     description="Project for Coverage Control",
#     author='Saurav Agarwal',
#     author_email='SauravAg@upenn.edu',
#     python_requires='>=3.10',
#     install_requires=install_requires,
#     ext_modules=get_extensions(),
#     cmdclass={
#         'build_ext':
#         BuildExtension.with_options(no_python_abi_suffix=True, use_ninja=False)
#     },
#     packages=find_packages(),
#     include_package_data=include_package_data,
# )

# # setup(name='pyCoverageControlExtensions',
# #       ext_modules=[cpp_extension.CppExtension('pyCoverageControlExtensions', ['extensions.cpp'])],
# #       cmdclass={'build_ext': cpp_extension.BuildExtension},
# #       include_dirs=['/root/CoverageControl_ws/install/include', '/usr/local/include/eigen3'],
# #       extra_compile_args={'cxx': ['-lCoverageControl -fopenmp -O3'] })
