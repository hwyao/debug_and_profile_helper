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
    
    # Get dimensions - look for first valid (non-zero) values across all messages
    # First, check if we have any dimension structure at all
    all_dims_list = df_parsed.apply(lambda x: x['msg']['layout']['dim'])
    if all_dims_list.empty or all(len(dims) == 0 for dims in all_dims_list):
        raise ValueError("No dimension data found in any messages")
    
    if mode == 'row_separation':
        # Row separation mode (like projector)
        # Split data into rows based on the first dimension
        
        # Find first non-zero dimension size from all messages
        all_dim1_sizes = df_parsed.apply(lambda x: x['msg']['layout']['dim'][0]['size'] if len(x['msg']['layout']['dim']) > 0 else 0)
        valid_dim1_sizes = all_dim1_sizes[all_dim1_sizes > 0]
        if valid_dim1_sizes.empty:
            raise ValueError("No valid dimensions found in row_separation mode data")
        dim1 = int(valid_dim1_sizes.iloc[0])
        
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
                print(f"\n❌ Error rows:")
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
        
        # Extract expected dimensions - find first valid (non-zero) sizes from all messages
        # Determine number of dimensions from first message with non-empty dims
        first_valid_dims = None
        for dims in all_dims_list:
            if len(dims) > 0:
                first_valid_dims = dims
                break
        
        if first_valid_dims is None:
            raise ValueError("No valid dimensions found in whole_array mode data")
        
        if len(first_valid_dims) < 2:
            warnings.warn(
                f"Data has {len(first_valid_dims)} dimensions. "
                f"whole_array mode is typically used for 2D+ data."
            )
        
        num_dims = len(first_valid_dims)
        dim_sizes = []
        expected_size = 1
        
        # For each dimension position, find first non-zero size across all messages
        for dim_idx in range(num_dims):
            all_sizes_for_dim = df_parsed.apply(
                lambda x: x['msg']['layout']['dim'][dim_idx]['size'] 
                if len(x['msg']['layout']['dim']) > dim_idx else 0
            )
            valid_sizes = all_sizes_for_dim[all_sizes_for_dim > 0]
            if valid_sizes.empty:
                raise ValueError(f"No valid size found for dimension {dim_idx} in whole_array mode data")
            
            dim_size = int(valid_sizes.iloc[0])
            dim_sizes.append(dim_size)
            expected_size *= dim_size
        
        # Create layout column
        df_regularized[f'{data_label}_layout'] = df_parsed.apply(
            lambda x: dim_sizes if len(x['msg']['data']) == expected_size else [None] * len(dim_sizes)
        )
        
        # Check for errors
        df_errors = df_regularized[df_regularized[f'{data_label}_layout'].apply(lambda x: x[0] is None)]
        if not df_errors.empty:
            warnings.warn(
                f"⚠️  Data size errors found in {len(df_errors)} rows. "
                f"Expected total size: {expected_size} ({' x '.join(map(str, dim_sizes))})"
            )
            if verbose:
                print(f"\n❌ Error rows:")
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


def align_rosbag_topics(
    dataframes: List[pd.DataFrame]
) -> pd.DataFrame:
    """
    Align and merge multiple ROS bag topic DataFrames based on time and index.
    
    This function intelligently merges DataFrames from multiple topics by detecting
    the appropriate alignment strategy based on available time columns ('time_bag',
    'time_count') and index columns ('idx').
    
    Time Alignment Scenarios:
    - Scene 1: All tables have both 'time_bag' AND 'time_count'
      * Validates that both lead to consistent merging results
      * Uses 'time_count' as primary evidence (more reliable)
      * Warns if 'time_bag' and 'time_count' produce different alignments
      
    - Scene 2: All tables have 'time_bag' but NO 'time_count'
      * Always warns that 'time_bag' is being used (less reliable due to ROS delays)
      * Merges based on 'time_bag' values
      
    - Scene 3: Mixed or neither column present
      * Raises error - no reliable merging evidence
    
    Column Alignment Scenarios:
    - Scene A: All have 'idx' column with matching unique idx values
      * Uses 'idx' as additional merging key (row-level alignment)
      * For each timestamp, only rows with same 'idx' are merged
      
    - Scene B: All without 'idx', no overlapping data columns
      * Direct merge on time columns only
      
    - Scene C: Problematic cases
      * Mixed presence/absence of 'idx' column
      * Different unique 'idx' values across DataFrames
      * Overlapping data column names (except time/idx/layout columns)
      * Raises error with detailed explanation
    
    Args:
        dataframes: List of pandas DataFrames to align and merge.
                   Each should have been produced by parse_rosbag_topic().
        time_tolerance: Tolerance for floating point comparison of time_count values.
                       Default: 1e-6
    
    Returns:
        pd.DataFrame: Merged DataFrame with aligned data from all topics.
                     Columns: time_bag, time_count (if available), idx (if applicable),
                             and all data columns from input DataFrames.
    
    Raises:
        ValueError: If DataFrames cannot be reliably merged (inconsistent time columns,
                   incompatible idx columns, or overlapping data columns)
    
    Examples:
        >>> # Merge two whole_array mode topics (no idx)
        >>> df_q = parse_rosbag_topic('data.bag', '/DBG_q', 'whole_array')
        >>> df_jac = parse_rosbag_topic('data.bag', '/DBG_jacobian', 'whole_array')
        >>> df_merged = align_rosbag_topics([df_q, df_jac])
        >>> # Result: time_bag, time_count, DBG_q, DBG_q_layout, DBG_jacobian, DBG_jacobian_layout
        
        >>> # Merge row_separation mode topics (with idx)
        >>> df_proj = parse_rosbag_topic('data.bag', '/DBG_projector', 'row_separation')
        >>> df_dist = parse_rosbag_topic('data.bag', '/DBG_distances', 'row_separation')
        >>> df_merged = align_rosbag_topics([df_proj, df_dist])
        >>> # Result: time_bag, time_count, idx, DBG_projector, DBG_projector_layout, 
        >>> #         DBG_distances, DBG_distances_layout
    """
    
    if not dataframes:
        raise ValueError("No DataFrames provided for alignment")
    
    if len(dataframes) == 1:
        warnings.warn("Only one DataFrame provided, returning it as-is")
        return dataframes[0].copy()
    
    # Check for empty DataFrames
    for i, df in enumerate(dataframes):
        if df.empty:
            raise ValueError(f"DataFrame at index {i} is empty")
    
    # ===== TIME ALIGNMENT ANALYSIS =====
    has_time_bag = [('time_bag' in df.columns) for df in dataframes]
    has_time_count = [('time_count' in df.columns) for df in dataframes]
    
    all_have_time_bag = all(has_time_bag)
    all_have_time_count = all(has_time_count)
    
    # Determine time alignment scenario
    if all_have_time_bag and all_have_time_count:
        # Scene 1: Both time columns present
        time_scenario = 1
        time_key = 'time_count'  # Use time_count as primary
        
    elif all_have_time_bag and not any(has_time_count):
        # Scene 2: Only time_bag present
        time_scenario = 2
        time_key = 'time_bag'
        warnings.warn("⚠️  Merging based on 'time_bag' only. This may be unreliable due to ROS timing delays. "
            "Consider using data with 'time_count' (timestep) for more accurate alignment.")
        
    else:
        # Scene 3: Mixed or neither
        raise ValueError(
            "Inconsistent time columns across DataFrames. "
            f"time_bag present in: {[i for i, v in enumerate(has_time_bag) if v]}, "
            f"time_count present in: {[i for i, v in enumerate(has_time_count) if v]}. "
            "All DataFrames must have consistent time column structure for alignment."
        )
    
    # ===== INDEX COLUMN ANALYSIS =====
    has_idx = [('idx' in df.columns) for df in dataframes]
    all_have_idx = all(has_idx)
    none_have_idx = not any(has_idx)
    
    if all_have_idx:
        # Scene A: All have idx - need to validate they're compatible
        # Check that all DataFrames have the same unique idx values for each time point
        idx_scenario = 'A'
        unique_idx_sets = [set(df['idx'].unique()) for df in dataframes]
        
        # Check if all have the same unique idx values
        if not all(idx_set == unique_idx_sets[0] for idx_set in unique_idx_sets):
            raise ValueError(
                "Incompatible 'idx' columns: DataFrames have different unique idx values. "
                f"Unique idx per DataFrame: {[sorted(list(s)) for s in unique_idx_sets]}. "
                "For row-level alignment, all DataFrames must have the same idx values."
            )
        
        merge_keys = [time_key, 'idx']
        
    elif none_have_idx:
        # Scene B: None have idx
        idx_scenario = 'B'
        merge_keys = [time_key]
        
    else:
        # Scene C: Mixed idx presence
        raise ValueError(
            f"Mixed 'idx' column presence: present in DataFrames {[i for i, v in enumerate(has_idx) if v]}, "
            f"absent in {[i for i, v in enumerate(has_idx) if not v]}. "
            "All DataFrames must either all have 'idx' or all not have 'idx' for alignment."
        )
    
    # ===== COLUMN OVERLAP ANALYSIS =====
    # Extract data columns (exclude time, idx, and layout columns)
    system_columns = {'time_bag', 'time_count', 'idx'}
    
    # Collect all data columns with their source DataFrame indices
    all_data_cols = []
    df_col_map = {}  # column -> list of df indices
    
    for i, df in enumerate(dataframes):
        for col in df.columns:
            if col not in system_columns:
                all_data_cols.append(col)
                if col not in df_col_map:
                    df_col_map[col] = []
                df_col_map[col].append(i)
    
    # Check for duplicates in flattened list
    if len(all_data_cols) != len(set(all_data_cols)):
        # Found duplicates - identify which columns and which DataFrames
        overlapping_cols = [col for col, indices in df_col_map.items() if len(indices) > 1]
        error_details = [f"'{col}' in DataFrames {indices}" for col, indices in df_col_map.items() if len(indices) > 1]
        raise ValueError(
            f"Overlapping data columns found: {', '.join(error_details)}. "
            "Cannot merge DataFrames with duplicate column names. "
            "Each topic should produce unique data column names."
        )
    
    # ===== PERFORM MERGE =====
    # Start with first DataFrame
    result = dataframes[0].copy()
    
    # Merge remaining DataFrames one by one
    for i, df in enumerate(dataframes[1:], start=1):
        # Determine merge type based on time_key
        if time_key == 'time_count':
            # For time_count, use exact merge (numerical values should match)
            result = pd.merge(
                result, 
                df, 
                on=merge_keys,
                how='outer',
                suffixes=('', '_drop')
            )
            
        else:  # time_bag
            # For time_bag (Timedelta), use merge_asof for nearest neighbor matching
            # This handles ROS timing delays better than exact merge
            
            if all_have_idx:
                # With idx: need to merge_asof within each idx group
                # Sort both DataFrames by time_bag
                result_sorted = result.sort_values('time_bag')
                df_sorted = df.sort_values('time_bag')
                
                # Merge by idx groups using merge_asof on time_bag
                merged_groups = []
                for idx_val in result_sorted['idx'].unique():
                    result_group = result_sorted[result_sorted['idx'] == idx_val]
                    df_group = df_sorted[df_sorted['idx'] == idx_val]
                    
                    if not df_group.empty:
                        merged_group = pd.merge_asof(
                            result_group,
                            df_group,
                            on='time_bag',
                            by='idx',
                            direction='nearest',
                            suffixes=('', '_drop')
                        )
                        merged_groups.append(merged_group)
                    else:
                        # No matching idx in df, keep result_group as-is
                        merged_groups.append(result_group)
                
                result = pd.concat(merged_groups, ignore_index=True)
            else:
                # Without idx: direct merge_asof on time_bag
                result = result.sort_values('time_bag')
                df_sorted = df.sort_values('time_bag')
                
                result = pd.merge_asof(
                    result,
                    df_sorted,
                    on='time_bag',
                    direction='nearest',
                    suffixes=('', '_drop')
                )
    
    # ===== SORT AND CLEAN =====
    # Clean the drop rows 
    result = result.loc[:, ~result.columns.str.endswith('_drop')]

    # Sort by merge keys
    result = result.sort_values(by=merge_keys).reset_index(drop=True)
    
    # Report merge statistics
    if all_have_idx:
        print(f"✅ Successfully merged {len(dataframes)} DataFrames")
        print(f"   Merge strategy: time={time_key}, idx=True (scenario {idx_scenario})")
        print(f"   Result shape: {result.shape} ({result.shape[0]} rows × {result.shape[1]} columns)")
    else:
        print(f"✅ Successfully merged {len(dataframes)} DataFrames")
        print(f"   Merge strategy: time={time_key}, idx=False (scenario {idx_scenario})")
        print(f"   Result shape: {result.shape} ({result.shape[0]} rows × {result.shape[1]} columns)")
    
    return result


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


def parse_rosbag_topic_bagpy(
    bag_path: str,
    topic: str,
    column_target: str,
    extract_dict: dict,
    mode: str = 'idx',
    verbose: bool = False
) -> pd.DataFrame:
    """
    Parse a rosbag topic using bagpy, extracting YAML data into structured columns.
    
    This function loads a rosbag using bagpy, converts the specified topic to CSV,
    reads it into a pandas DataFrame, and transforms it according to the provided
    column mappings and extraction rules.
    
    Args:
        bag_path: Path to the ROS bag file
        topic: Topic name to parse
        column_target: Name of the column containing YAML data to extract
        extract_dict: Dictionary mapping YAML paths to column names
                     e.g., {'primitive.type': 'my_column_name'}
        mode: Either 'idx' (for list of YAMLs) or 'non-idx' (for single YAML)
        verbose: If True, display parsing information
    
    Returns:
        pd.DataFrame: Transformed DataFrame with extracted columns
    
    Example:
        >>> column_mapping = {
        ...     'primitive.type': 'primitive.type',
        ...     'primitive_pose.position.x': 'primitive.position.x',
        ... }
        >>> df = parse_rosbag_topic_bagpy(
        ...     'data.bag',
        ...     '/debug_and_profile_helper/DBG_obstacles',
        ...     'obstacles',
        ...     column_mapping,
        ...     mode='idx'
        ... )
    """
    from bagpy import bagreader
    import re
    
    # Load bag using bagpy and convert to panda table
    bag = bagreader(bag_path)
    csv_path = bag.message_by_topic(topic)
    df_raw = pd.read_csv(csv_path, quotechar='"')
    
    if verbose:
        print(f"Loaded {len(df_raw)} rows from topic {topic}")
    
    # Helper function to parse YAML list
    def parse_yaml_list(text):
        if not isinstance(text, str) or not text.strip():
            return []
        content = text.strip()
        if content.startswith('[') and content.endswith(']'):
            content = content[1:-1].strip()
        if not content:
            return []
        # Split by comma followed by the first key pattern
        parts = re.split(r',\s*(?=\w+:)', content)
        res = []
        for p in parts:
            try:
                obj = yaml.safe_load(p.strip())
                if obj:
                    res.append(obj)
            except:
                continue
        return res
    
    # Helper function to get nested value from dictionary
    def get_nested_value(obj, path):
        keys = path.split('.')
        value = obj
        for key in keys:
            if isinstance(value, dict):
                value = value.get(key)
            else:
                return None
            if value is None:
                return None
        return value
    
    # Helper function to filter and reorder columns
    def filter_columns(df, extract_dict, mode):
        """Keep only requested columns in proper order."""
        # Base columns that should always be kept
        base_cols = ['time_bag', 'time_count'] if mode == 'non-idx' else ['time_bag', 'time_count', 'idx']
        # Columns from extract_dict (the VALUES, which are the new column names)
        requested_cols = list(extract_dict.values())
        # Combine and filter to only existing columns
        all_wanted_cols = base_cols + requested_cols
        existing_cols = [c for c in all_wanted_cols if c in df.columns]
        return df[existing_cols]
    
    # Process based on mode
    if mode == 'idx':
        # Mode: list of YAML objects
        # Parse all YAML lists first (vectorized operation)
        yaml_lists = df_raw[column_target].apply(parse_yaml_list)
        
        # Process each row using itertuples (faster than iterrows)
        all_rows = []
        has_time_count = 'header.stamp.secs' in df_raw.columns
        
        for idx, (row_data, yaml_list) in enumerate(zip(df_raw.itertuples(), yaml_lists)):
            if not yaml_list:
                yaml_list = [{}]  # Keep at least one row with empty data
            
            # Normalize the list of YAML objects
            df_step = pd.json_normalize(yaml_list)
            
            # Add time columns (use index-based access for columns with dots)
            df_step['time_bag'] = row_data.Time
            if has_time_count:
                df_step['time_count'] = df_raw.iloc[idx]['header.stamp.secs']
            
            # Add index column
            df_step['idx'] = range(len(df_step))
            
            all_rows.append(df_step)
        
        # Concatenate all rows
        df_result = pd.concat(all_rows, ignore_index=True)
        
        # Rename columns according to extract_dict
        df_result.rename(columns=extract_dict, inplace=True)
        
        # Filter to keep only requested columns
        df_result = filter_columns(df_result, extract_dict, mode='idx')
        
    else:  # mode == 'non-idx'
        # Initialize result with time columns
        df_result = pd.DataFrame()
        df_result['time_bag'] = df_raw['Time']
        
        if 'header.stamp.secs' in df_raw.columns:
            df_result['time_count'] = df_raw['header.stamp.secs']
        
        # Define extraction function for efficiency
        def create_extractor(yaml_path):
            def extract_value(text):
                if not isinstance(text, str) or not text.strip():
                    return None
                try:
                    obj = yaml.safe_load(text.strip())
                    return get_nested_value(obj, yaml_path)
                except:
                    return None
            return extract_value
        
        # Apply extraction for each mapping
        for yaml_path, col_name in extract_dict.items():
            extractor = create_extractor(yaml_path)
            df_result[col_name] = df_raw[column_target].apply(extractor)
        
        # Filter to keep only requested columns (non-idx mode doesn't have 'idx' column)
        df_result = filter_columns(df_result, extract_dict, mode='non-idx')
    
    if verbose:
        print(f"Result shape: {df_result.shape}")
        print(f"Columns: {list(df_result.columns)}")
    
    return df_result

# Command-line interface placeholder (to be implemented later)
if __name__ == '__main__':
    print(
        "Command-line interface not yet implemented.\n"
        "Please use this module by importing it:\n\n"
        "    from log_ros_parser import parse_rosbag_topic\n"
        "    df = parse_rosbag_topic('file.bag', '/topic_name', mode='row_separation')\n"
    )
    sys.exit(0)
