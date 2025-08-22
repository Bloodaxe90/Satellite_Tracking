import os
import datetime
import pandas as pd


def save_results(
    root_dir: str, results: pd.DataFrame, exp_name: str, final: bool = False
):
    """
    Save a pandas DataFrame as a CSV file to a specific directory

    Args:
        root_dir (str): The root directory
        results (pd.DataFrame): DataFrame containing the results to save
        exp_name (str): Experiment name which is also the  filename (with or without .csv extension)
        final (bool): Determines if the file should be loaded from logs or results (Defaults to False)
              (i.e. final results would be saved with final as True)

    """
    print("Saving Results")
    # Create directory path with current date
    if final:
        result_dir = f"{root_dir}/results"
    else:
        result_dir = f"{root_dir}/logs/{datetime.datetime.now().date()}"
    os.makedirs(result_dir, exist_ok=True)

    # Ensure file extension is .csv
    if ".csv" not in exp_name:
        exp_name += ".csv"

    # Save results
    results.to_csv(f"{result_dir}/{exp_name}", index=False)
    print(f"Saved {exp_name} in directory: {result_dir}\n")


def load_results(root_dir: str, exp_name: str, date_dir: str = "") -> pd.DataFrame:
    """
    Load a CSV results file from a specified directory

    Args:
        root_dir (str): The root directory
        date_dir (str): The subdirectory named by date (Defaults to empty string)
        exp_name (str): Experiment filename (with or without .csv extension)

    Returns:
        pd.DataFrame: Loaded DataFrame from the CSV file
    """
    print("Loading Results")
    # Build directory path for given date, if not date_dir is given load results from result directory
    if date_dir:
        result_dir = f"{root_dir}/logs/{date_dir}"
    else:
        result_dir = f"{root_dir}/results"

    os.makedirs(result_dir, exist_ok=True)  # Create if not exists

    # Ensure file extension is .csv
    if ".csv" not in exp_name:
        exp_name += ".csv"

    print(f"Loaded {exp_name} from directory: {result_dir}")
    return pd.read_csv(f"{result_dir}/{exp_name}")
