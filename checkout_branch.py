#!/usr/bin/python3

import os
import argparse
from typing import List

def checkout_repo_branch(repo: str, branch: str):
    if not os.system(f"cd {repo} && git pull origin {branch} && cd .."):
        return
    
    raise Exception("Failed to checkout repo")

def checkout_main_repos_branch(branch: str):
    main_repos: List[str] = [
        "roboteam_ai",
        "roboteam_proto",
        "roboteam_robothub",
        "roboteam_utils",
        "roboteam_world"
    ]
    for each in main_repos:
        checkout_repo_branch(each, branch)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Checkout the main roboteam branches")
    parser.add_argument("branch_name", type=str, help='The branch to checkout the repos on')

    args = parser.parse_args()
    checkout_main_repos_branch(args.branch_name)