import pandas as pd
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

file_path = r'C:\Users\Sam Butler\OneDrive\Desktop\Boids\Boids-project\parameter_sweep_results.csv'
df = pd.read_csv(file_path) #accesses cv file

np.random.seed(42) #creates random seed

indices = np.random.permutation(len(df)) #create random indices for split
split_point = int(len(df) * 0.7)#creates 70/30 split
training_idx, validation_idx = indices[:split_point], indices[split_point:] #creates reference to training validation values from random split


training_set = df.iloc[training_idx]#creates training set from reference
validation_set = df.iloc[validation_idx]#creates validation set from reference
training_set.to_csv('training_set_70percent.csv', index=False)
validation_set.to_csv('validation_set_30percent.csv', index=False)
# loads split datasets
training_set_df = pd.read_csv('training_set_70percent.csv')
validation_set_df = pd.read_csv('validation_set_30percent.csv')

print("Training set descriptive statistics:")
print(training_set_df.describe())
# Correlation matrix for training set
pd.set_option('display.max_columns', None)
pd.set_option('display.width', 1000)
np.set_printoptions(linewidth=1000)
print("\nTraining Set Correlation Matrix:")
correlation_matrix = training_set_df.corr()
print(correlation_matrix)

def perform_validation_tests(training_df, validation_df, x, y):
    # creates relationship on training data
    train_corr, train_p = stats.pearsonr(training_df[x], training_df[y])
    slope, intercept, r_value, p_value, std_e = stats.linregress(training_df[x], training_df[y])
    train_r2 = r_value**2

    y_pred = intercept + slope * validation_df[x] #uses prediciton on regression
    val_corr, val_p = stats.pearsonr(validation_df[x], validation_df[y])#calculates validation metrics
    mae = mean_absolute_error(validation_df[y], y_pred)
    rmse = np.sqrt(mean_squared_error(validation_df[y], y_pred))
    val_r2 = r2_score(validation_df[y], y_pred)
    
    print(f"\n{x} vs {y} - Validation Results:")#outputs metrics
    print("Training Model:")
    print(f"  Correlation: r = {train_corr:.4f}, p = {train_p:.4f}")
    print(f"  Regression Equation: {y} = {intercept:.4f} + {slope:.4f} × {x}")
    print(f"  Training R²: {train_r2:.4f}")
    
    print("Validation Performance:")
    print(f"  Validation Correlation: r = {val_corr:.4f}, p = {val_p:.4f}")
    print(f"  Mean Absolute Error: {mae:.4f}")
    print(f"  Root Mean Square Error: {rmse:.4f}")
    print(f"  Validation R²: {val_r2:.4f}")
    
    return {
        'relationship': f"{x} vs {y}",
        'training_correlation': train_corr,
        'training_p_value': train_p,
        'training_r2': train_r2,
        'slope': slope,
        'intercept': intercept,
        'validation_correlation': val_corr,
        'validation_p_value': val_p,
        'validation_mae': mae,
        'validation_rmse': rmse,
        'validation_r2': val_r2
    }

validation_results = []
relationships = [
    ('nnd', 'num_schools'),
    ('visual_range', 'num_schools'),
    ('field_of_view', 'nnd'),
    ('visual_range', 'polarisation')
]

for x, y in relationships: 
    result = perform_validation_tests(training_set_df, validation_set_df, x, y)
    validation_results.append(result)
