#!/usr/bin/env python3
"""
Log File Parser - Convert structured log files to CSV or other formats

This script parses log files with the following format:
[timestamp] [file_logger] [level] [file] variable_name: data

Supported features:
1. File format validation
2. Convert to CSV format (extensible to support other formats)
3. Handle cyclic data and separators
4. Warning and NaN filling when data is missing
"""

import argparse
import csv
import os
import re
import sys
import warnings
from typing import List, Optional, Tuple
import pandas as pd

# Supported output formats configuration
SUPPORTED_FORMATS = {
    'csv': '.csv'
    # Future formats can be added here, e.g.:
    # 'json': '.json',
    # 'xlsx': '.xlsx'
}

# Create reverse mapping for extension to format lookup
EXTENSION_TO_FORMAT = {v: k for k, v in SUPPORTED_FORMATS.items()}


class LogFileParser:
    def __init__(self, suppress_warnings: bool = False, max_warnings: int = 50, verbose: bool = False):
        """
        Initialize log file parser
        
        Args:
            suppress_warnings: Whether to suppress warning messages
            max_warnings: Maximum number of warnings before stopping execution
            verbose: Whether to print verbose output
        """
        self.suppress_warnings = suppress_warnings
        self.max_warnings = max_warnings
        self.verbose = verbose
        self.warning_count = 0
        self.log_pattern = re.compile(
            r'\[([^\]]+)\] \[([^\]]+)\] \[([^\]]+)\] \[([^\]]+)\] ([^:]+):\s*(.*)'
        )  # Matches: [timestamp] [logger] [level] [file] variable_name: data
        self.continuation_pattern = re.compile(
            r'^[\s\-\d\.eE\+]+$'
        )  # Matches: numeric data lines (whitespace, digits, scientific notation)
        self.data_records = []
        self.variable_names = set()
        self.cycles = []
        
        # Metrics
        self.metrics = {
            'total_lines': 0,
            'empty_lines': 0,
            'valid_lines': 0,
            'continuation_lines': 0,
            'unacceptable_lines': 0,
        }
        
    def _log(self, message: str, force: bool = False):
        """Print message if verbose or forced"""
        if self.verbose or force:
            print(message)

    def _warn(self, message: str, category=UserWarning):
        """Issue warning (if not suppressed) and check warning count"""
        self.warning_count += 1
        if self.warning_count > self.max_warnings:
            raise RuntimeError(f"Too many warnings ({self.warning_count}). Maximum allowed: {self.max_warnings}. Stopping execution to prevent further issues.")
        
        if not self.suppress_warnings:
            warnings.warn(message, category)
    
    def _trigger_temp_data_collection(self, temp_data: List[str], temp_timestamp: str, temp_var_name: str, frozen_ts: Optional[float] = None) -> dict:
        """
        Process temp_data and return data record for appending to data_records
        
        Args:
            temp_data: List of data strings to be combined
            temp_timestamp: Timestamp for the data record
            temp_var_name: Variable name for the data record
            frozen_ts: Optional frozen timestep
            
        Returns:
            Dict containing the data record
        """
        combined_data = ' ; '.join(temp_data) if len(temp_data) > 1 else temp_data[0]
        return {
            'timestamp': temp_timestamp,
            'variable_name': temp_var_name,
            'value': combined_data,
            'frozen_ts': frozen_ts
        }
    
    def parse_file(self, file_path: str) -> pd.DataFrame:
        """
        Parse structured log text file into DataFrame.
        

        Args:
            file_path: Log file path
            
        Returns:
            pd.DataFrame: DataFrame with columns [timestamp, variable_name, value]
        """
        total_lines = 0
        empty_lines = 0
        valid_lines = 0
        continuation_lines = 0
        data_records = []
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        except Exception as e:
            raise IOError(f"Unable to read file: {e}")
        
        line_idx = 0
        match = None
        match_next = None
        temp_data = []
        temp_timestamp = None
        temp_var_name = None
        temp_frozen_ts = None # Initialize

        # Add sentinel line to trigger final data collection
        sentinel_line = "[END] [END] [END] [END] END: END"
        lines.append(sentinel_line)

        while line_idx < len(lines):
            if not lines[line_idx].strip():
                line_idx += 1
                empty_lines += 1
                continue

            # Count total lines if not empty
            total_lines += 1

            match_next = self.log_pattern.match(lines[line_idx])
            if match_next:   # New log entry found
                # Save accumulated data from previous entry
                if temp_data:
                    record = self._trigger_temp_data_collection(temp_data, temp_timestamp, temp_var_name, temp_frozen_ts)
                    data_records.append(record)
                    temp_data = []
                    temp_frozen_ts = None # Reset

                # Initialize new entry
                match = match_next
                temp_timestamp, logger, level, file_info, temp_var_name, data = match.groups()
                
                # Check for frozen timestep prefix: "[123]; data"
                frozen_ts = None
                frozen_match = re.match(r'^\[([\d\.]+)\];\s*(.*)$', data.strip())
                if frozen_match:
                    frozen_ts = float(frozen_match.group(1))
                    data = frozen_match.group(2)
                
                temp_data = [data.strip()]
                # Update logic for expected_cols. If lines are matrix rows, they should match first line.
                expected_cols = len(data.strip().split())
                
                # Store frozen_ts with the record (need to update _trigger_temp_data_collection or pass it somehow)
                # But _trigger_temp_data_collection is called later.
                # Use a temp variable for frozen_ts
                temp_frozen_ts = frozen_ts # Store
                valid_lines += 1

            elif self.continuation_pattern.match(lines[line_idx]): # Continuation line found
                # Validate continuation line requirements
                if not temp_data:
                    self._warn(f"Line {line_idx + 1}: Found continuation line without preceding log entry: {lines[line_idx].strip()[:50]}...")
                    temp_data = []
                elif len(lines[line_idx].strip().split()) != expected_cols:
                    self._warn(
                        f"Line {line_idx + 1}: Found continuation line with mismatched columns, "
                        f"expect {expected_cols}; actual {len(lines[line_idx].strip().split())}. :{lines[line_idx].strip()[:50]}..."
                    )
                    temp_data = []
                else:
                    # Valid continuation line - append to current entry
                    temp_data.append(lines[line_idx].strip())
                    continuation_lines += 1

            else: # Invalid line format - first trigger previous data collection, then clear and warn
                self._warn(f"Line {line_idx + 1} format does not comply with specification: {lines[line_idx].strip()[:50]}...")
                if temp_data:
                    record = self._trigger_temp_data_collection(temp_data, temp_timestamp, temp_var_name, temp_frozen_ts)
                    data_records.append(record)
                    temp_data = []
                    temp_frozen_ts = None

            # increase the data
            line_idx += 1

        # Clean up sentinel line and adjust counters
        assert lines[-1] == sentinel_line, "INTERNAL PROGRAMMING ERROR: Sentinel line is missing or incorrect."
        lines.pop()
        total_lines -= 1
        valid_lines -= 1

        if total_lines == 0:
            raise ValueError("File is empty")

        if empty_lines > 0:
            self._warn(f"Found {empty_lines} empty lines")

        # Calculate statistics
        acceptable_lines = valid_lines + continuation_lines
        format_ratio = acceptable_lines / total_lines
        
        # Store metrics
        self.metrics.update({
            'total_lines': total_lines,
            'empty_lines': empty_lines,
            'valid_lines': valid_lines,
            'continuation_lines': continuation_lines,
            'unacceptable_lines': total_lines - acceptable_lines
        })

        # Report line statistics
        self._log(f"\n📊 Format validation and parsing results:")
        self._log(f"   {total_lines} Total lines scanned. [+ {empty_lines} empty lines]")
        self._log(f"   {acceptable_lines} Acceptable lines.")
        self._log(f"    - {valid_lines} Valid log lines.")
        self._log(f"    - {continuation_lines} Continuation lines.")
        self._log(f"   {total_lines - acceptable_lines} Unacceptable lines with warning.")
        self._log(f"   Format compliance ratio: {format_ratio:.1%}\n")

        if format_ratio < 0.9:  # At least 90% of lines should be acceptable
            self._warn(f"File format validation failed: only {format_ratio:.1%} of lines match expected format")
            if not data_records:
                raise ValueError("No valid data found in file")
        
        # Convert to DataFrame
        if not data_records:
            raise ValueError("No valid data records found")
            
        df = pd.DataFrame(data_records)
        self._log(f"✅ Successfully parsed {len(df)} data records. (Should match valid lines: {valid_lines})")
        self._log(f"📈 There are {df['variable_name'].nunique()} unique variables")
        return df
    
    def to_csv(self, output_path: str, df: pd.DataFrame, dummy_separator: Optional[str] = None, force_overwrite: bool = False) -> dict:
        """
        Convert parsed DataFrame to CSV format with cycle detection and ordering
        
        Args:
            output_path: Output CSV file path
            df: Parsed DataFrame with columns [timestamp, variable_name, value]
            dummy_separator: Cycle separator identifier
            force_overwrite: Whether to overwrite existing files
            
        Returns:
            dict: Conversion metrics
        """
        if df.empty:
            self._warn("No data available for conversion")
            return
        
        # Check if output file already exists
        if os.path.exists(output_path):
            if not force_overwrite:
                raise FileExistsError(f"Output file already exists: {output_path}. Use --force to overwrite.")
            else:
                self._warn(f"Overwriting existing file: {output_path}")
        
        
        # Process cycles based on dummy_separator
        # Helper: check is frozen_ts exists in df
        has_frozen_ts = 'frozen_ts' in df.columns and df['frozen_ts'].notna().any()
        cycle_method = "unknown"

        # Process cycles based on frozen_ts (priority 1) or dummy_separator (priority 2)
        if has_frozen_ts:
            cycle_method = "frozen_timestep"
            self._log("ℹ️  Using frozen timestep for cycle alignment.")
            # Use frozen_ts as cycle_id directly (cast to int if it is effectively int)
            # Or rank them to get cycle_id 1, 2, 3...
            # But frozen_ts might not be monotonic if files are concatenated? Assuming monotonic.
            # We can just use frozen_ts as the cycle identifier.
            df['cycle_id'] = df['frozen_ts']
            
            # Since explicit cycle ID is available, we assume no separator processing needed for cycle logic.
            # But we should still remove the dummy separator row if it exists?
            if dummy_separator:
                df_cycle_marked = df[df['variable_name'] != dummy_separator].copy()
                unique_variable_names = [var for var in df['variable_name'].unique() if var != dummy_separator]
            else:
                df_cycle_marked = df.copy()
                unique_variable_names = df['variable_name'].unique().tolist()
                
            separator_count = 0 # Not using separator for count

        elif dummy_separator:
            cycle_method = "dummy_separator"
            # Find separator indices
            separator_mask = df['variable_name'] == dummy_separator
            separator_indices = df[separator_mask].index.tolist()

            # Save statistics
            separator_count = len(separator_indices)
            unique_variable_names = df['variable_name'].drop_duplicates().tolist()

            # Initialize cycle_id and process DataFrame
            df['cycle_id'] = 1
            separator_indices = [-1] + separator_indices
            if separator_indices[-1] < len(df):
                separator_indices = separator_indices + [len(df)]

            for i in range(len(separator_indices) - 1):
                # Assign the cycle_id
                start_idx = separator_indices[i] + 1
                end_idx = separator_indices[i + 1]
                df.loc[start_idx:end_idx, 'cycle_id'] = i + 1
                group = df.iloc[start_idx:end_idx]
                
                # Check for duplicates
                cycle_variable_names = group['variable_name'].tolist()
                if len(cycle_variable_names) != len(set(cycle_variable_names)):
                    raise ValueError(f"Cycle {i + 1} contains duplicate variable names.")

                # Check order alignment
                aligned = all(
                    unique_variable_names.index(var) <= unique_variable_names.index(next_var)
                    for var, next_var in zip(cycle_variable_names, cycle_variable_names[1:])
                    if var in unique_variable_names and next_var in unique_variable_names
                )
                if not aligned:
                    warnings.warn(f"Cycle {i + 1} has variable names out of order: {cycle_variable_names}")

            # Remove separator rows
            df_cycle_marked = df[~separator_mask].copy()
            unique_variable_names = [var for var in unique_variable_names if var != dummy_separator]
        else:
            cycle_method = "none"
            # Check if separator-like variables exist and warn
            separator_vars = [var for var in df['variable_name'].unique() if 'separator' in var.lower()]
            if separator_vars:
                self._warn(f"Detected possible separator variables {separator_vars}, but --dummy_separator not specified")
            if not has_frozen_ts:
                raise NotImplementedError(f"Automatic separator detection is not yet implemented. Please specify --dummy_separator or use logs with frozen timestep.")
        
        
        # Get unique variables in order of first appearance
        unique_cycles = sorted(df['cycle_id'].unique())
        
        self._log(f"📊 Data processing statistics:")
        self._log(f"   {len(df)} Total records")
        self._log(f"   - {len(df_cycle_marked)} Processed records")
        self._log(f"   - {separator_count} Separator count (dummy separator: {dummy_separator})")
        self._log(f"   {len(unique_cycles)} Unique cycles")
        self._log(f"   {len(unique_variable_names)} Variables (remove separator, appearance order): \n {', '.join(unique_variable_names)}\n")

        # Create pivot table to convert to cycle-based CSV format
        # Each row represents a cycle, each column represents a variable
        try:
            pivot_df = df_cycle_marked.pivot_table(
                index='cycle_id', 
                columns='variable_name', 
                values='value', 
                aggfunc='first'  # Take first value if duplicates exist
            )
            
            # Reorder columns to match appearance order
            # Assert that unique_variable_names and pivot_df.columns are equivalent
            assert set(unique_variable_names) == set(pivot_df.columns), (
                f"INTERNAL ERROR: unique_variable_names and pivot_df.columns mismatch.\n"
                f"unique_variable_names: {unique_variable_names}\n"
                f"pivot_df.columns: {list(pivot_df.columns)}"
            )

            # Assign new index for this table
            pivot_df.reset_index(inplace=True)

            # Reorder columns: cycle_id, then variables in appearance order
            column_order = ['cycle_id'] + unique_variable_names
            pivot_df = pivot_df[column_order]
            
            # Calculate statistics before filling NaN
            total_cells = len(unique_cycles) * len(unique_variable_names)
            data_cells = pivot_df[unique_variable_names].notna().sum().sum()
            missing_cells = total_cells - data_cells
            
            # Find missing data positions
            missing_positions = []
            for cycle_id in unique_cycles:
                cycle_data = pivot_df[pivot_df['cycle_id'] == cycle_id][unique_variable_names].iloc[0]
                for col_idx, var in enumerate(unique_variable_names):
                    if pd.isna(cycle_data[var]):
                        missing_positions.append(f"({cycle_id},{col_idx+1})")
            
            self._log(f"📊 Pivot table statistics:")
            self._log(f"   {total_cells} = {len(unique_cycles)} × {len(unique_variable_names)} Full Size")
            self._log(f"   {data_cells} Available data")
            if missing_cells > 0:
                self._log(f"   Missing data:")
                self._log(f"{', '.join(missing_positions[:10])}{'...' if len(missing_positions) > 10 else ''}\n")
            
            # Fill NaN values with 'NaN' string for CSV output
            pivot_df = pivot_df.fillna('NaN')
            
        except Exception as e:
            raise ValueError(f"Error creating pivot table: {e}")
        
        # Write to CSV file
        try:
            pivot_df.to_csv(output_path, index=False)
            self._log(f"📁 Output saved to: {output_path}")
            self._log(f"📈 CSV format: {len(unique_cycles)} rows × {len(column_order)} columns")
            
            return {
                'processed_records': len(df_cycle_marked),
                'separator_count': separator_count,
                'unique_cycles': len(unique_cycles),
                'csv_rows': len(unique_cycles),
                'available_data': data_cells,
                'csv_cols': len(column_order),
                'cycle_method': cycle_method,
            }
        except Exception as e:
            raise IOError(f"Unable to write CSV file: {e}")


def validate_output_and_format(output_path: Optional[str], convert_to: str) -> Tuple[str, str]:
    """
    Validate output path and format consistency
    
    Args:
        output_path: User specified output path (can be None)
        convert_to: Target conversion format
        
    Returns:
        Tuple[str, str]: (validated_output_path, validated_format)
        
    Raises:
        ValueError: If there's a conflict between output file extension and convert_to format
    """
    if output_path:
        # Extract file extension if present
        if '.' in output_path:
            base_path, ext = output_path.rsplit('.', 1)
            file_ext = '.' + ext.lower()
        else:
            base_path = output_path
            file_ext = None
        
        # Check if file extension matches a supported format
        if file_ext and file_ext in EXTENSION_TO_FORMAT:
            # Extension corresponds to a supported format
            inferred_format = EXTENSION_TO_FORMAT[file_ext]
            
            if convert_to != inferred_format:
                # Conflict between --output extension and --convert_to
                raise ValueError(
                    f"Conflict detected: output file extension '{file_ext}' suggests format '{inferred_format}', "
                    f"but --convert_to specifies '{convert_to}'. "
                    f"Please ensure consistency or remove the file extension from --output."
                )
            
            return output_path, convert_to
        else:
            # No extension or extension not recognized, append appropriate extension
            expected_ext = SUPPORTED_FORMATS.get(convert_to)
            if expected_ext:
                return output_path + expected_ext, convert_to
            else:
                return output_path, convert_to
    
    return None, convert_to


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Convert structured log files to CSV or other formats',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Example usage:
  %(prog)s log_file.txt
  %(prog)s log_file.txt --convert_to csv --dummy_separator "DBG_dummy_separator"
  %(prog)s log_file.txt -s
  %(prog)s log_file.txt -o output.csv --force
        '''
    )
    
    parser.add_argument('input_file', help='Input log file path')
    parser.add_argument('--output', '-o', type=str,
                       help='Output file path (default: generated based on input filename)')
    parser.add_argument('--convert_to', '-2', choices=list(SUPPORTED_FORMATS.keys()), default='csv',
                       help='Target conversion format (default: csv)')
    parser.add_argument('--dummy_separator', '-s', type=str,
                       help='Separator identifier between cycles')
    parser.add_argument('--suppress-warnings', '-Wno', action='store_true',
                       help='Suppress all warning messages')
    parser.add_argument('--force', '-f', action='store_true',
                       help='Force overwrite existing output files')
    
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    try:
        if args.verbose:
            print("=" * 60)
            print(f"{'INITIALIZATION':^60}")
            print("=" * 60)
        else:
            print(f"Running in Summary Mode (use --verbose for full output)")
        
        # Validate output path and format consistency
        validated_output, validated_format = validate_output_and_format(args.output, args.convert_to)
        
        # Generate output filename if not provided
        if validated_output:
            output_file = validated_output
        else:
            input_base = args.input_file.rsplit('.', 1)[0]
            ext = SUPPORTED_FORMATS.get(validated_format)
            assert ext is not None, (
                f"INTERNAL PROGRAMMING ERROR: Unsupported format '{validated_format}' not found in SUPPORTED_FORMATS. "
                f"This indicates a bug in the validation logic or format configuration. "
                f"Available formats: {list(SUPPORTED_FORMATS.keys())}"
            )
            output_file = f"{input_base}{ext}"
        
        if args.verbose:
            print(f"Input file:  {args.input_file}")
            print(f"Output file: {output_file}")
            print(f"Format:      {validated_format.upper()}")
            if args.dummy_separator:
                print(f"Separator:   '{args.dummy_separator}'")
            print()
            
            print("=" * 60)
            print(f"{'PARSING':^60}")
            print("=" * 60)
            print(f"Reading and parsing log file: {args.input_file}\n")
        
        # Create parser
        parser_instance = LogFileParser(suppress_warnings=args.suppress_warnings, verbose=args.verbose)
        df = parser_instance.parse_file(args.input_file)
        
        if df.empty:
            print("⚠️  Warning: No valid data found")
            sys.exit(1)
        if args.verbose:
            print()

            print("=" * 60)
            print(f"{'CONVERSION':^60}")
            print("=" * 60)
            
            # distribute the df table to conversion functions
            print(f"Converting to {validated_format.upper()} format...\n")
            
        conversion_metrics = {}
        if validated_format == 'csv':
            conversion_metrics = parser_instance.to_csv(output_file, df, dummy_separator=args.dummy_separator, force_overwrite=args.force)
        
        if args.verbose:
            print()
            print("=" * 60)
            print(f"{'COMPLETED':^60}")
            print("=" * 60)
        
        # --- Validation & Summary ---
        if not conversion_metrics:
            if args.verbose:
                print("🎉 Conversion completed successfully")
            return

        # Gather metrics
        metrics = parser_instance.metrics.copy()
        
        # Add conversion metrics
        metrics['parsed_records'] = len(df)
        metrics['total_records'] = len(df)
        metrics.update(conversion_metrics)
        
        # Validation Checks
        all_passed = True
        validation_output = []
        
        def check(condition, message):
            status = "[OK]" if condition else "[FAILED]"
            validation_output.append(f"    {status} {message}")
            return condition

        validation_output.append("  Validation:")
        
        # Check 1: unacceptable_lines <= 1
        all_passed &= check(metrics['unacceptable_lines'] <= 1, 
                           f"Unacceptable lines: {metrics['unacceptable_lines']} (≦ 1)")
        
        # Check 2: parsed_records = valid_lines = total_records
        all_passed &= check(metrics['parsed_records'] == metrics['valid_lines'] == metrics['total_records'],
                           f"Parsed records: {metrics['parsed_records']} = Valid lines: {metrics['valid_lines']} = Total records: {metrics['total_records']}")
        
        # Check 3: processed_records = available_data
        all_passed &= check(metrics['processed_records'] == metrics['available_data'],
                           f"Processed records: {metrics['processed_records']} = Available data: {metrics['available_data']}")
        
        cycle_method = metrics.get('cycle_method', 'unknown')
        
        # Check 4: Cycle Logic (Conditional)
        if cycle_method == 'dummy_separator':
            sep = metrics['separator_count']
            uniq = metrics['unique_cycles']
            all_passed &= check(sep <= uniq <= sep + 1,
                               f"Unique cycles: {uniq} (Separator count: {sep}, diff: {uniq - sep}) [Method: {cycle_method}]")
        elif cycle_method == 'frozen_timestep':
             all_passed &= check(metrics['unique_cycles'] >= 1,
                                f"Unique cycles: {metrics['unique_cycles']} >= 1 [Method: {cycle_method}]")
        
        # Check 5: unique_cycles = csv_rows
        all_passed &= check(metrics['unique_cycles'] == metrics['csv_rows'],
                           f"Unique cycles: {metrics['unique_cycles']} = CSV rows: {metrics['csv_rows']}")

        if 'csv_cols' in metrics:
             validation_output.append(f"    [INFO] CSV columns: {metrics['csv_cols']}")

        # Print Conclusion
        if args.verbose:
            for line in validation_output:
                print(line)
            if all_passed:
                print("\n🎉 Conversion completed successfully!")
            else:
                print("\n⚠️  Success but validation failed")
        else:
            # Short summary for non-verbose
            if all_passed:
                 print(f"✅ Success - All checks passed (Method: {cycle_method})")
            else:
                 print(f"⚠️  Success but validation failed (Method: {cycle_method})")
                 for line in validation_output:
                     print(line)

        print()
        
    except Exception as e:
        print()
        print("=" * 60)
        print(f"{'ERROR':^60}")
        print("=" * 60)
        print(f"❌ Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()