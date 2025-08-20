#Importing required libraries
library(tidyverse)
library(googlesheets4)
library(googledrive)
library(janitor)
library(broom)
library(anytime)

#Authenticating the google account
gs4_auth()

#Setting the plot theme
theme_set(theme_classic())

#Reading in the raw data from Google Sheets Curves Sheet
raw_data <- read_sheet("https://docs.google.com/spreadsheets/d/12PiDsrHu31zx2Lrlpn-d5TqE_Kz8P-uKUGBP9FFGlAk/edit?usp=sharing", sheet = "Data_raw_new") 

#Fixing the timestamp to POSIXct format from UNIX
clean_data <- raw_data |>
  mutate(Timestamp = anytime(Timestamp))

#Filter by required the datapoints and plot the data of Moisture Release Curves
clean_data |>
  ggplot() + geom_point(aes(x=Timestamp, y=AirTemp_C))
