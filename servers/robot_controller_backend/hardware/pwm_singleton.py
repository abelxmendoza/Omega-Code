"""
PWM Singleton Module

Thread-safe singleton pattern for PCA9685 instances.
Ensures only one PCA9685 instance exists per process, shared across all components.
"""

from threading import Lock

_pca_instance = None
_pca_lock = Lock()

def get_pca(PCAClass, *args, **kwargs):
    """
    Get or create singleton PCA9685 instance.
    
    Args:
        PCAClass: PCA9685 class (real or mock)
        *args: Positional arguments for PCA9685 constructor
        **kwargs: Keyword arguments for PCA9685 constructor
    
    Returns:
        Singleton PCA9685 instance
    """
    global _pca_instance
    with _pca_lock:
        if _pca_instance is None:
            _pca_instance = PCAClass(*args, **kwargs)
        return _pca_instance

def reset_pca():
    """Reset singleton instance (useful for testing)."""
    global _pca_instance
    with _pca_lock:
        _pca_instance = None

