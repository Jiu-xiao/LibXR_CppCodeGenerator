# src/libxr/libxr_package_info.py

import logging

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

class LibXRPackageInfo:
    """
    libxr pip package self-check utility:
    - Prints local version
    - Checks for new remote version and suggests upgrading
    """
    PKGNAME = "libxr"

    @classmethod
    def get_local_version(cls):
        try:
            from importlib.metadata import version, PackageNotFoundError
        except ImportError:
            from importlib_metadata import version, PackageNotFoundError
        try:
            return version(cls.PKGNAME)
        except PackageNotFoundError:
            return None

    @classmethod
    def get_remote_version(cls):
        try:
            import requests
        except ImportError:
            return None
        try:
            resp = requests.get(f"https://pypi.org/pypi/{cls.PKGNAME}/json", timeout=3)
            if resp.status_code == 200:
                return resp.json()["info"]["version"]
        except Exception:
            pass
        return None

    @classmethod
    def check_and_print(cls):
        local_ver = cls.get_local_version()
        remote_ver = cls.get_remote_version()
        if local_ver:
            logging.info(f"{cls.PKGNAME} {local_ver}")
        else:
            logging.warning(f"{cls.PKGNAME} (version unknown)")
        if local_ver and remote_ver:
            try:
                from packaging.version import parse as vparse
                if vparse(local_ver) < vparse(remote_ver):
                    logging.warning(f"A new version of {cls.PKGNAME} is available: {remote_ver} (your version: {local_ver})")
                    logging.warning(f"Tip: Upgrade with: pip install -U {cls.PKGNAME}\n")
            except Exception:
                # fallback: only show if different
                if local_ver != remote_ver:
                    logging.warning(f"A new version of {cls.PKGNAME} is available: {remote_ver} (your version: {local_ver})")
                    logging.warning(f"Tip: Upgrade with: pip install -U {cls.PKGNAME}\n")
