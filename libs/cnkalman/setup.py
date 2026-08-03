import glob
import os
import pathlib
import shutil
import sys

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext as build_ext_orig

cmake_path = os.environ.get('CMAKE_PATH', "cmake")
class CMakeExtension(Extension):

    def __init__(self, name):
        # don't invoke the original build_ext for this special extension
        super().__init__(name, sources=[])


class build_ext(build_ext_orig):

    def run(self):
        for ext in self.extensions:
            self.build_cmake(ext)
        super().run()

    def build_cmake(self, ext):
        cwd = pathlib.Path().absolute()

        # these dirs will be created in build_py, so if you don't have
        # any python sources to bundle, the dirs will be missing
        build_temp = pathlib.Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)
        extdir = pathlib.Path(self.get_ext_fullpath(ext.name))
        extdir.mkdir(parents=True, exist_ok=True)

        config = 'Release'
        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + str(extdir.parent.absolute()),
            '-DCMAKE_BUILD_TYPE=' + config,
            '-DCMAKE_INSTALL_PREFIX=' + str(build_temp.absolute()),
            '-DPYTHON_EXECUTABLE=' + sys.executable
        ]

        # example of build args
        build_args = [
            '--config', config,
        ]

        build_temp_dir = str(build_temp.absolute())
        ext_dir_parent = str(extdir.parent.absolute())
        os.chdir(str(build_temp.absolute()))
        self.spawn([cmake_path, str(cwd)] + cmake_args)
        if not self.dry_run:
            self.spawn([cmake_path, '--build', '.', '--target', 'install'] + build_args)
            print(build_temp_dir + "/lib/*.so")
            for file in glob.glob(build_temp_dir + "/lib/*.so"):
                print(file)
                shutil.copy(file, ext_dir_parent + "/cnkalman")

        # Troubleshooting: if fail on line above then delete all possible
        # temporary CMake files including "CMakeCache.txt" in top level dir.
        os.chdir(str(cwd))


setup(
    name='cnkalman',
    packages=['cnkalman'],
    ext_modules=[CMakeExtension('.')],
    cmdclass={
        'build_ext': build_ext,
    },
    setup_requires=["setuptools-git-versioning"],
)