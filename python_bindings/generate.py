#!/usr/bin/env python3
import os
import argparse
from collections import OrderedDict

try:
    # For generating bindings
    import cffi 
except (ImportError):
    print("[!] `CFFI` is missing.")
    exit(1)

# pycparser is a dependency of cffi, so must be available here
from pycparser import parse_file, c_generator, c_ast 

# The C pre-processor expands the fake stdlib headers we provide. If this is compiled by cffi, there will be weird type size issues.
def kill_unwanted_typedefs(tree):
    for x, i in tree.children():
        # We only need to look at Typedefs
        if type(i) is c_ast.Typedef:
            # If the type is too simple or if it's a struct with no implementation
            if type(i.type.type) is c_ast.IdentifierType or (type(i.type.type) is c_ast.Struct and not i.type.type.decls):
                tree.ext.remove(i)
            # Some edge cases - a typedef with pointers
            elif type(i.type) is c_ast.PtrDecl:
                if type(i.type.type.type) is c_ast.IdentifierType:
                    tree.ext.remove(i)


def expose_decl_in_bindings(files: list, ffi: cffi.FFI, c_compiler: str):
    # Create a temporary file as workaround for issues with redeclaring types
    with open('.tmp_preprocess.h', 'w') as tmp_file:
        tmp_file.write(format_include_for_compiler(files))
    
    # Converts an AST representation to C code. Used to create cdef-compatible type/func declarations.
    generator = c_generator.CGenerator()
    
    res = parse_file(os.path.abspath('.tmp_preprocess.h'), use_cpp=True,  cpp_path=c_compiler, cpp_args=[r'-E', r'-nostdinc', r'-Ipycparser/utils/fake_libc_include'])
    
    kill_unwanted_typedefs(res)
    
    # Remove function bodies
    for x, i in enumerate(res.ext):
        # If we find a function definition
        if type(i) is c_ast.FuncDef:
            res.ext[x] = i.decl 
    
    # Cdef specifies what should be exposed in the final bindings
    ffi.cdef(generator.visit(res))

    # Remove created tmp file
    os.remove('.tmp_preprocess.h')

def format_include_for_compiler(files: list) -> str:
    # This could probably be done with a lambda
    final = ""
    for file in files:
        final += f"#include \"{os.path.abspath(file)}\"\n"
    return final

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generates python bindings for given files')
    parser.add_argument("--includes", required=True, nargs='+', help="C header files to make bindings for")
    parser.add_argument("--output", nargs=1, type=str, help="Output location of the python bindings", default=".")
    parser.add_argument("--name", nargs=1, type=str, help="Name of the generated python module", default="_roboteam_embedded_messages")
    parser.add_argument("--lib", nargs='+', help="Additional libraries to link with", default=[])
    parser.add_argument("--compiler", nargs=1, type=str, help="C(++) compiler to use to preprocess files", default="gcc")
    args = parser.parse_args()

    ffi = cffi.FFI()
    expose_decl_in_bindings(args.includes, ffi, c_compiler=args.compiler)
    ffi.set_source(args.name, format_include_for_compiler(args.includes), libraries=args.lib)

    ffi.compile(tmpdir=args.output)

    print("[+] Bindings generated!")