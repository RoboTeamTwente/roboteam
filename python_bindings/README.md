# Python bindings generator for roboteam_embedded_messages

## Usage

1. (Optional) Enter a python virtual env
2. Run setup.sh to setup the enviornemnt
    * Clone the pycparser repo for fake stdlib headers
    * Install python libraries specified in requirements.txt
3. Run python3 generate.py -h for help
    * All options have defaults, so they will run without any changes.

## Requirements

* cffi >= 1.14.4
* pycparser (a dependency of cffi that's also used by generate.py)

## Important notice

This implementation may be incompatible with certain typedefs, however such incompatible typedefs aren't currently generated and can only be introduced with future changes. If for some reason a typedef is being omitted by this generator, either the original C code should be changed or this implementation refactored. 