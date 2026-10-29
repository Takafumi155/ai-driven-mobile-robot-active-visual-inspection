from setuptools import setup
setup(name='second_look',version='0.2.0',py_modules=['world','runtime','runner'],data_files=[('share/ament_index/resource_index/packages',['resource/second_look']),('share/second_look',['package.xml'])],entry_points={'console_scripts':['inspection = runner:main']})
