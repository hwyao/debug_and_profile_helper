#!/usr/bin/env python3
"""
ROS Bag Parser - Parse Float64Array messages from ROS bag files to pandas DataFrames

This module parses ROS bag files containing std_msgs/Float64MultiArray messages
and converts them into pandas DataFrames. It automatically detects timestep data
(when first dimension label is 'timestep') and supports two parsing modes:
1. Row separation mode: Separates 2D data into individual rows (like projector)
2. Whole array mode: Keeps data as complete arrays (like jacobian)

Requirements:
- Python 3.10+ (required for rosbags library)
- rosbags
- pandas
- numpy
- yaml

Usage as a module:
    from log_ros_parser import parse_rosbag_topic
    
    df = parse_rosbag_topic(
        bag_path='path/to/file.bag',
        topic='/debug_and_profile_helper/DBG_projector',
        mode='row_separation',
        verbose=True
    )

Author: Generated for debug_and_profile_helper
Date: 2026-01-26
"""

import sys
import warnings
from typing import Optional, Literal, List, Dict, Any
import pandas as pd
import numpy as np
import yaml

# Check Python version
if sys.version_info < (3, 10):
    raise RuntimeError(
        f"ROS bag parsing requires Python 3.10 or later (rosbags library requirement). "
        f"Current version: {sys.version_info.major}.{sys.version_info.minor}. "
        f"Please use Python 3.10+ to run this module."
    )

try:
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError as e:
    raise ImportError(
        "Required package 'rosbags' is not installed. "
        "Please install it with: pip install rosbags"
    ) from e


def parse_rosbag_topic(
    bag_path: str,
    topic: str,
    mode: Literal['row_separation', 'whole_array'] = 'row_separation',
    verbose: bool = False
) -> pd.DataFrame:
    """
    Parse a Float64Array topic from a ROS bag file into a pandas DataFrame.
    
    This function reads ROS bag messages and automatically detects whether the data
    includes a timestep (by checking if the first dimension label is 'timestep').
    It supports two parsing modes:
    
    - 'row_separation': Separates multi-dimensional data into individual rows.
      Example: A 10x7 matrix is exploded into 10 rows, each containing a 7-element vector.
      Use case: Time series of vectors (e.g., projector data)
      
    - 'whole_array': Keeps multi-dimensional data as complete arrays.
      Example: A 6x7 matrix is kept as a single 2D array.
      Use case: Matrices that should stay together (e.g., Jacobian)
    
    Args:
        bag_path: Path to the ROS bag file
        topic: Topic name to parse (must be Float64MultiArray type)
        mode: Parsing mode - 'row_separation' or 'whole_array'
        verbose: If True, display parsing statistics and information.
                If False, only display warnings for problems and critical errors.
    
    Returns:
        pd.DataFrame: Parsed data with columns:
            - time_bag: Timestamp from bag file (pd.Timedelta)
            - time_count: Timestep value (if timestep detected, otherwise NaN)
            - Other columns based on mode:
                * row_separation: 'idx', data column, layout column
                * whole_array: data column, layout column
    
    Raises:
        FileNotFoundError: If bag file doesn't exist
        ValueError: If topic not found or has wrong message type
        RuntimeError: If parsing encounters critical errors
    
    Examples:
        >>> # Parse projector data (row separation mode)
        >>> df_proj = parse_rosbag_topic(
        ...     'data.bag',
        ...     '/debug_and_profile_helper/DBG_projector',
        ...     mode='row_separation'
        ... )
        >>> # Result: Each row is a 7D vector, indexed by (time_bag, time_count, idx)
        
        >>> # Parse Jacobian data (whole array mode)
        >>> df_jac = parse_rosbag_topic(
        ...     'data.bag',
        ...     '/debug_and_profile_helper/DBG_jacobian',
        ...     mode='whole_array'
        ... )
        >>> # Result: Each row contains a full 6x7 matrix
    """
    
    # Validate mode
    if mode not in ['row_separation', 'whole_array']:
        raise ValueError(f"Invalid mode '{mode}'. Must be 'row_separation' or 'whole_array'")
    
    # Get typestore for ROS message deserialization
    typestore = get_typestore(Stores.ROS1_NOETIC)
    
    # Statistics
    total_messages = 0
    skipped_messages = 0
    has_timestep = None  # Will be determined from first message
    
    # Data collection
    records = []
    
    try:
        with Reader(bag_path) as reader:
            # Find connections for the specified topic
            connections = [x for x in reader.connections if x.topic == topic]
            
            if not connections:
                available_topics = [c.topic for c in reader.connections]
                raise ValueError(
                    f"Topic '{topic}' not found in bag file. "
                    f"Available topics: {', '.join(available_topics)}"
                )
            
            # Verify message type
            msg_type = connections[0].msgtype
            if 'Float64MultiArray' not in msg_type:
                raise ValueError(
                    f"Topic '{topic}' has type '{msg_type}', but this parser only supports "
                    f"'std_msgs/Float64MultiArray' messages."
                )
            
            # Parse messages
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                total_messages += 1
                
                try:
                    msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                    arr = msg.data
                    
                    # Detect timestep on first message
                    if has_timestep is None:
                        has_timestep = (len(msg.layout.dim) > 0 and 
                                      msg.layout.dim[0].label == 'timestep')
                        if verbose:
                            timestep_status = "WITH timestep" if has_timestep else "WITHOUT timestep"
                            print(f"ℹ️  Detected data format: {timestep_status}")
                            print(f"   Message type: {msg_type}")
                            print(f"   Topic: {topic}")
                    
                    # Extract timestep if present
                    time_count = None
                    data_dims = msg.layout.dim
                    
                    if has_timestep:
                        time_count = arr[0]
                        arr = arr[1:]
                        data_dims = msg.layout.dim[1:]
                    
                    # Store message data as YAML for intermediate processing
                    # (Following the pattern from data_visualization.ipynb)
                    data_yaml = yaml.dump({
                        'msg': {
                            'layout': {
                                'dim': [
                                    {'label': d.label, 'size': d.size}
                                    for d in data_dims
                                ]
                            },
                            'data': arr.tolist()
                        }
                    })
                    
                    records.append({
                        'time_bag': pd.Timedelta(timestamp),
                        'time_count': time_count,
                        f'data_yaml': data_yaml
                    })
                    
                except Exception as e:
                    skipped_messages += 1
                    if verbose:
                        warnings.warn(f"Skipped message at timestamp {timestamp}: {e}")
                    continue
    
    except FileNotFoundError:
        raise FileNotFoundError(f"Bag file not found: {bag_path}")
    except Exception as e:
        raise RuntimeError(f"Error reading bag file: {e}")
    
    # Check if we got any data
    if not records:
        raise ValueError(f"No valid messages found for topic '{topic}'")
    
    # Convert to initial DataFrame
    df = pd.DataFrame(records)
    
    if verbose:
        print(f"\n📊 Parsing statistics:")
        print(f"   {total_messages} total messages")
        print(f"   {total_messages - skipped_messages} successfully parsed")
        if skipped_messages > 0:
            print(f"   {skipped_messages} skipped due to errors")
    
    # Parse YAML data and regularize based on mode
    df_parsed = df['data_yaml'].apply(yaml.safe_load)
    
    # Extract data label from topic name (e.g., '/debug.../DBG_projector' -> 'DBG_projector')
    data_label = topic.split('/')[-1]
    
    # Create regularized DataFrame
    df_regularized = df[['time_bag', 'time_count']].copy()
    df_regularized[data_label] = df_parsed.apply(lambda x: x['msg']['data'])
    
    # Get dimensions from first message
    first_dims = df_parsed.iloc[0]['msg']['layout']['dim']
    
    if mode == 'row_separation':
        # Row separation mode (like projector)
        # Split data into rows based on the first dimension
        
        if len(first_dims) == 0:
            raise ValueError("Cannot use row_separation mode with 0-dimensional data")
        
        dim1 = first_dims[0]['size']
        
        # Reshape data into list of rows
        df_regularized[data_label] = df_regularized[data_label].apply(
            lambda x: [x[i:i + dim1] for i in range(0, len(x), dim1)]
        )
        
        # Handle empty data
        df_regularized[data_label] = df_regularized[data_label].apply(
            lambda x: [[]] if (not isinstance(x, list) or len(x) == 0) else x
        )
        
        # Explode into separate rows
        df_regularized = df_regularized.explode(data_label)
        
        # Add row index within each message
        df_regularized['idx'] = df_regularized.groupby(level=0).cumcount()
        
        # Create layout column for validation
        df_regularized[f'{data_label}_layout'] = df_regularized[data_label].apply(
            lambda x: [1, dim1] if len(x) == dim1 else ([0, dim1] if len(x) == 0 else [None, dim1])
        )
        
        # Check for errors
        df_errors = df_regularized[df_regularized[f'{data_label}_layout'].apply(lambda x: x[0] is None)]
        if not df_errors.empty:
            warnings.warn(
                f"⚠️  Data dimension errors found in {len(df_errors)} rows. "
                f"Expected dimension: {dim1}, but got different sizes."
            )
            if verbose:
                print(f"\n❌ Error rows detected:")
                print(df_errors)
        
        # Reset index and reorder columns
        df_regularized = df_regularized[[
            'time_bag', 'time_count', 'idx', data_label, f'{data_label}_layout'
        ]].reset_index(drop=True)
        
        if verbose:
            print(f"\n📈 Row separation mode:")
            print(f"   Dimension per row: {dim1}")
            print(f"   Total rows after separation: {len(df_regularized)}")
            print(f"   Unique time points: {df_regularized['time_bag'].nunique()}")
    
    elif mode == 'whole_array':
        # Whole array mode (like jacobian)
        # Keep data as complete N-dimensional arrays
        
        if len(first_dims) < 2:
            warnings.warn(
                f"Data has {len(first_dims)} dimensions. "
                f"whole_array mode is typically used for 2D+ data."
            )
        
        # Extract expected dimensions
        expected_size = 1
        dim_sizes = []
        for dim in first_dims:
            dim_sizes.append(dim['size'])
            expected_size *= dim['size']
        
        # Create layout column
        df_regularized[f'{data_label}_layout'] = df_parsed.apply(
            lambda x: dim_sizes if len(x['msg']['data']) == expected_size else [None] * len(dim_sizes)
        )
        
        # Check for errors
        df_errors = df_regularized[df_regularized[f'{data_label}_layout'].apply(lambda x: x[0] is None)]
        if not df_errors.empty:
            warnings.warn(
                f"⚠️  Data size errors found in {len(df_errors)} rows. "
                f"Expected total size: {expected_size} ({' × '.join(map(str, dim_sizes))})"
            )
            if verbose:
                print(f"\n❌ Error rows detected:")
                print(df_errors)
        
        # Reset index and reorder columns
        df_regularized = df_regularized[[
            'time_bag', 'time_count', data_label, f'{data_label}_layout'
        ]].reset_index(drop=True)
        
        if verbose:
            print(f"\n📈 Whole array mode:")
            print(f"   Array dimensions: {' × '.join(map(str, dim_sizes))}")
            print(f"   Total data points: {len(df_regularized)}")
    
    if verbose:
        print(f"\n✅ Parsing complete!")
        print(f"   Output shape: {df_regularized.shape}")
        print(f"   Columns: {list(df_regularized.columns)}")
    
    return df_regularized


def get_available_topics(bag_path: str) -> List[Dict[str, str]]:
    """
    Get list of available topics in a bag file.
    
    Args:
        bag_path: Path to the ROS bag file
    
    Returns:
        List of dicts with 'topic' and 'msgtype' keys
    
    Example:
        >>> topics = get_available_topics('data.bag')
        >>> for t in topics:
        ...     print(f"{t['topic']}: {t['msgtype']}")
    """
    try:
        with Reader(bag_path) as reader:
            return [
                {'topic': c.topic, 'msgtype': c.msgtype}
                for c in reader.connections
            ]
    except FileNotFoundError:
        raise FileNotFoundError(f"Bag file not found: {bag_path}")
    except Exception as e:
        raise RuntimeError(f"Error reading bag file: {e}")


# Command-line interface placeholder (to be implemented later)
if __name__ == '__main__':
    print(
        "Command-line interface not yet implemented.\n"
        "Please use this module by importing it:\n\n"
        "    from log_ros_parser import parse_rosbag_topic\n"
        "    df = parse_rosbag_topic('file.bag', '/topic_name', mode='row_separation')\n"
    )
    sys.exit(0)
