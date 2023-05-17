from setuptools import setup, Extension
from torch.utils import cpp_extension

setup(name='pyCoverageControl_extras',
      ext_modules=[cpp_extension.CppExtension('Maps', ['torch_ext.cpp'])],
      cmdclass={'build_ext': cpp_extension.BuildExtension},
      include_dirs=['/home/saurav/opt/CoverageControl_ws/install/include'],
      extra_compile_args={'cxx': ['-lCoverageControl -O3'] })
