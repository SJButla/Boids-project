import pandas as pd
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats

file_path = r'C:\Users\Sam Butler\OneDrive\Desktop\Boids\Boids-project\parameter_sweep_results.csv'
df = pd.read_csv(file_path)

print(df.describe())
#correlation matrix
pd.set_option('display.max_columns', None)
pd.set_option('display.width', 1000)
np.set_printoptions(linewidth=1000)
print("\nCorrelation Matrix:") 
correlation_matrix = df.corr() #outputs correlation matrix of all ocmbinations of metrics and parameters
print(correlation_matrix)


#visual range vs schools
plt.figure(figsize=(8, 6)) #creates boxplot of visual range vs number of schools
sns.boxplot(x='visual_range', y='num_schools', data=df)
plt.title('Distribution of Number of Schools by Visual Range')
plt.xlabel('Visual Range')
plt.ylabel('Number of Schools')
plt.tight_layout()
plt.savefig('visual_range_vs_schools.png') #saves as png
plt.close()

# polarisation vs visual range
plt.figure(figsize=(8, 6)) #creates boxplot of polarisation vs visual range
sns.boxplot(x='visual_range', y='polarisation', data=df)
plt.title('Distribution of Polarisation by Visual Range')
plt.xlabel('Visual Range')
plt.ylabel('Polarisation')
plt.tight_layout()
plt.savefig('visual_range_vs_polarisation.png') #saves as png
plt.close()

#nnd vs schools
plt.figure(figsize=(8, 6))
plt.scatter(df['nnd'], df['num_schools']) #creates scatter of nearest neighbour distance vs number of schools- continuous quantitatve data
plt.title('Nearest Neighbour Distance vs Number of Schools')
plt.xlabel('Nearest Neighbour Distance')
plt.ylabel('Number of Schools')
plt.tight_layout()
plt.savefig('nnd_vs_schools.png') #saves as png
plt.close()

#field of view vs nnd
plt.figure(figsize=(8, 6)) #creates boxplot of field of view vs nearest neighbour distance
sns.boxplot(x='field_of_view', y='nnd', data=df)
plt.title('Distribution of Nearest Neighbour Distance by Field of View')
plt.xlabel('Field of View')
plt.ylabel('Nearest Neighbour Distance')
plt.tight_layout()
plt.savefig('field_of_view_vs_nnd.png')# saves as png
plt.close()




def perform_statistical_tests(df, x, y):
    # pearson Correlation
    correlation, p_value = stats.pearsonr(df[x], df[y])
    
    print(f"\n{x} vs {y}:")
    print("Pearson Correlation:")
    print(f"  Correlation Coefficient: {correlation:.4f}")
    print(f"  P-value: {p_value:.4f}")
    
    # linear Regression
    slope, intercept, r_value, p_value, std_e= stats.linregress(df[x], df[y])
    
    print("Linear Regression:")
    print(f"  Slope: {slope:.4f}")
    print(f"  Intercept: {intercept:.4f}")
    print(f"  R-squared: {r_value**2:.4f}")
    print(f"  P-value: {p_value:.4f}")

# performs tests for each relationship
perform_statistical_tests(df, 'nnd', 'num_schools')
perform_statistical_tests(df, 'visual_range', 'num_schools')
perform_statistical_tests(df, 'field_of_view', 'nnd')
perform_statistical_tests(df, 'visual_range', 'polarisation')