from setuptools import setup, find_packages

setup(
    name="Sorter",
    packages=find_packages(),
    scripts=["bin/ts_master", "bin/ts_camera_reader", "bin/ts_image_saver"],
    data_files=[
        ("/etc/supervisor/conf.d", ["supervisor/thingsorter.conf"]),
        ("/etc", ["thingsorter.conf"]),
    ],
    include_package_data=True,
)
