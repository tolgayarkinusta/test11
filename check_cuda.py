"""Check CUDA availability and system paths."""
import sys
import torch

def check_cuda():
    """Verify CUDA is available."""
    cuda_available = torch.cuda.is_available()
    print(f'CUDA available: {cuda_available}')
    return cuda_available

def print_python_paths():
    """Print Python system paths."""
    print("\nPython paths:")
    for path in sys.path:
        print(path)

if __name__ == '__main__':
    check_cuda()
    print_python_paths()
