# Python bindings generator for roboteam_embedded_messages

## Usage

1. (Optional) Enter a python virtual env
2. Run bash.sh to setup the enviornemnt
    * Clone the pycparser repo for fake stdlib headers
    * Install python libraries specified in requirements.txt
3. Run python3 generate.py -h for help
    * All options have defaults, so they will run without any changes.

## Requirements

* cffi >= 1.14.4
* pycparser (a dependency of cffi that's also used by generate.py)
