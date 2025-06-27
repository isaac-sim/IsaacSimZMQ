# SPDX-FileCopyrightText: Copyright (c) 2019-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#

import json
import logging
import os
import platform
from pathlib import Path

logger = logging.getLogger(__name__)

REPO_ROOT = os.path.join(os.path.dirname(os.path.normpath(__file__)), "../..")
REPO_CACHE_FILE = os.path.join(REPO_ROOT, "repo-cache.json")


def repoman_bootstrap():
    _path_checks()
    _prep_cache_paths()


def _path_checks():
    """Check for problematic path conditions and warn appropriately."""
    cwd = os.getcwd()
    if " " in cwd:
        logger.warning(
            "Current working directory: %s contains whitespace which may cause issues with some tooling such as premake within repo_build. It is recommended to move your project to a path without spaces.",
            cwd,
        )

    # Check if current working directory is within a OneDrive folder
    if platform.system() == "Windows":
        onedrive_path = os.getenv("OneDrive")  # For personal OneDrive
        onedrive_business_path = os.getenv("OneDriveCommercial")  # For business accounts

        if not onedrive_path and not onedrive_business_path:
            # OneDrive is not installed or synced
            return

        if (onedrive_path and cwd.startswith(onedrive_path)) or (
            onedrive_business_path and cwd.startswith(onedrive_business_path)
        ):
            logger.warning(
                "Current working directory: %s appears to be within a OneDrive folder. This may cause filesystem issues with Packman linking dependencies. It is recommended to move your project outside of OneDrive.",
                cwd,
            )


def _prep_cache_paths():
    """
    There are several environment variables that repo_man can optionally set to control where various caches are placed. They will all be relative to the repository root.
    - PM_PACKAGES_ROOT: this is where Packman stores its package cache
    - PIP_CACHE_DIR: this is where pip stores its wheel cache
    - UV_CACHE_DIR: this is where uv stores its wheel and package cache

    There are several gating flags as well to prevent repo_man from using the pip/uv default cache dir envvars unless explicitly set by us.
    - OM_PIP_CACHE: gating pip cache dir flag for omni.repo.man.deps.pip_install_requirements
    - OM_UV_CACHE: gating uv cache dir flag for omni.repo.man.deps._uv_requirements_load
    """

    repo_cache_file = Path(REPO_CACHE_FILE)
    if repo_cache_file.is_file():
        # cache file is present, read it in and set environment variables.
        cache_path_data = json.loads(repo_cache_file.read_text())
        # resolve REPO_ROOT rather than relative path to avoid any chdir shenanigans.
        resolved_root = Path(REPO_ROOT).resolve()

        for cache, cache_path in cache_path_data.items():
            # Expand $HOME and ~
            resolved_path = Path(os.path.expandvars(os.path.expanduser(cache_path)))
            if not resolved_path.is_dir():
                # Relative path to current working directory or absolute path is not present.
                # It's possible repo was somehow executed outside of the repository root.
                resolved_path = resolved_root / cache_path

            # Fully resolve path to avoid weird dir popping in some workflows.
            os.environ[cache] = resolved_path.resolve().as_posix()
            resolved_path.mkdir(parents=True, exist_ok=True)

            # Set repo_man breadcrumb to respect PIP_CACHE_DIR and UV_CACHE_DIR.
            # Unset OMNI_REPO_ROOT to force the caching of installed Python deps
            # in the packman cache dir.
            if cache == "PIP_CACHE_DIR":
                os.environ["OM_PIP_CACHE"] = "1"
                os.environ["OMNI_REPO_ROOT"] = ""
            elif cache == "UV_CACHE_DIR":
                os.environ["OM_UV_CACHE"] = "1"
                os.environ["OMNI_REPO_ROOT"] = ""
