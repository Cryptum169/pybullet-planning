import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="PyBullet-Planning",
    version="1.0.0",
    author="Caelan Garrett",
    author_email="author@example.com",
    description="PyBullet Simulator for PR2 Robots",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/cryptum169/pybullet-planning",
    project_urls={
        "Bug Tracker": "https://github.com/cryptum169/pybullet-planning/issues",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    package_dir={"": "."},
    packages=setuptools.find_packages(where="."),
    python_requires=">=3.6",
)
