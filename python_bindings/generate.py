#!/usr/bin/env python3
import os
import argparse
import tempfile

try:
    # For generating bindings
    import cffi 
except ImportError:
    print("[!] `CFFI` is missing.")
    exit(1)

# pycparser is a dependency of cffi, so must be available here
from pycparser import parse_file, c_generator, c_ast 

# The C pre-processor expands the fake stdlib headers we provide. If this is compiled by cffi, there will be weird isinstance size issues.
def kill_unwanted_typedefs(tree):
    for _, i in tree.children():
        # We only need to look at Typedefs
        if not isinstance(i, c_ast.Typedef):
            continue
        # If the isinstance is too simple or if it's a struct with no implementation
        if isinstance(i.type.type, c_ast.IdentifierType) or (isinstance(i.type.type, c_ast.Struct) and not i.type.type.decls):
            tree.ext.remove(i)
        # Some edge cases - a typedef with pointers
        elif isinstance(i.type, c_ast.PtrDecl):
            if isinstance(i.type.type.type, c_ast.IdentifierType):
                tree.ext.remove(i)

def remove_function_bodies(tree):
    for x, i in enumerate(tree.ext):
        # If we find a function definition
        if isinstance(i, c_ast.FuncDef):
            # Make it a declaration
            tree.ext[x] = i.decl 

def expose_decl_in_bindings(files: list, ffi: cffi.FFI, c_compiler: str):
    # Create a temporary file as workaround for issues with redeclaring types
    tmp_file = tempfile.NamedTemporaryFile(mode='w',suffix='.h')

    tmp_file.write(format_include_for_compiler(files))
    tmp_file.flush()
    
    # Converts an AST representation to C code. Used to create cdef-compatible isinstance/func declarations.
    generator = c_generator.CGenerator()
    
    res = parse_file(tmp_file.name, use_cpp=True,  cpp_path=c_compiler, cpp_args=[r'-E', r'-nostdinc', r'-Ipycparser/utils/fake_libc_include'])
    
    kill_unwanted_typedefs(res)
    remove_function_bodies(res)

    # Cdef specifies what should be exposed in the final bindings
    ffi.cdef(generator.visit(res))

    tmp_file.close()


def format_include_for_compiler(files: list) -> str:
    # This could probably be done with a lambda
    final = ""
    for file in files:
        final += f"#include \"{os.path.abspath(file)}\"\n"
    return final

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generates python bindings for given files')
    parser.add_argument("--includes", nargs='+', required=True, help="C header files to make bindings for")
    parser.add_argument("--output", type=str, help="Output location of the python bindings", default=".")
    parser.add_argument("--name", type=str, help="Name of the generated python module", default="_roboteam_embedded_messages")
    parser.add_argument("--lib", nargs='+', help="Additional libraries to link with", default=[])
    parser.add_argument("--compiler", type=str, help="C(++) compiler to use to preprocess files", default="gcc")
    args = parser.parse_args()

    ffi = cffi.FFI()
    expose_decl_in_bindings(args.includes, ffi, c_compiler=args.compiler)
    ffi.set_source(args.name, format_include_for_compiler(args.includes), libraries=args.lib)

    ffi.compile(tmpdir=args.output)

    print("[+] Bindings generated!")
