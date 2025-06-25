import time
import functools
from typing import TypeVar, Callable, Any, ParamSpec

# Type variables for generic function signatures
P = ParamSpec('P')  # For arbitrary parameters
R = TypeVar('R')    # For return type

def time_function(func: Callable[P, R]) -> Callable[P, R]:
    """
    A decorator that times the execution of a function with arbitrary inputs and outputs.
    
    Args:
        func: The function to be timed
        
    Returns:
        A wrapped function that prints timing information and returns the original result
        
    Example:
        @time_function
        def my_function(x: int, y: str) -> bool:
            time.sleep(0.1)
            return True
            
        result = my_function(42, "hello")  # Prints timing info and returns True
    """
    @functools.wraps(func)
    def wrapper(*args: P.args, **kwargs: P.kwargs) -> R:
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        
        elapsed_time = end_time - start_time
        print(f"Function '{func.__name__}' took {elapsed_time:.6f} seconds to execute")
        
        return result
    
    return wrapper


def time_function_with_return(func: Callable[P, R]) -> Callable[P, tuple[R, float]]:
    """
    A decorator that times the execution of a function and returns both the result and timing.
    
    Args:
        func: The function to be timed
        
    Returns:
        A wrapped function that returns (original_result, elapsed_time)
        
    Example:
        @time_function_with_return
        def my_function(x: int) -> str:
            time.sleep(0.1)
            return f"Result: {x}"
            
        result, elapsed = my_function(42)  # Returns ("Result: 42", 0.1)
    """
    @functools.wraps(func)
    def wrapper(*args: P.args, **kwargs: P.kwargs) -> tuple[R, float]:
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        
        elapsed_time = end_time - start_time
        return result, elapsed_time
    
    return wrapper


# Example usage and testing
if __name__ == "__main__":
    @time_function
    def example_function(x: int, y: str, z: bool = True) -> str:
        """Example function with various parameter types."""
        time.sleep(0.1)  # Simulate some work
        return f"Processed {x}, {y}, {z}"
    
    @time_function_with_return
    def example_function_with_return(x: float) -> list[float]:
        """Example function that returns timing information."""
        time.sleep(0.05)  # Simulate some work
        return [x * 2, x * 3, x * 4]
    
    # Test the decorators
    print("Testing time_function decorator:")
    result1 = example_function(42, "hello", False)
    print(f"Result: {result1}\n")
    
    print("Testing time_function_with_return decorator:")
    result2, elapsed = example_function_with_return(10.5)
    print(f"Result: {result2}")
    print(f"Elapsed time: {elapsed:.6f} seconds") 