#!/usr/bin/cython
from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension
from Cython.Distutils import build_ext

setup(
    name = 'tracker',
    ext_modules=[
    Extension('test',
            sources=['tracker.pyx'],
            extra_compile_args=['-O3'],
            language='c++')
    ],
    cmdclass = {'build_ext': build_ext}
    # ext_modules = cythonize("*.pyx"),
)