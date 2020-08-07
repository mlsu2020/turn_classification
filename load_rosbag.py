import pandas
import yaml
import rosbag_pandas

file = '../beta_autorally4_2018-03-29-11-50-25.bag'
dataframe = rosbag_pandas.bag_to_dataframe(file)
print(dataframe)
