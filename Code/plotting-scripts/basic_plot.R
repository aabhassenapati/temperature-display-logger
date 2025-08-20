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

#Fixing the timestamp to POSIXct format from UNIX, the default recording timezone for the device in UNIX is in UTC, although it is actually mapped to time in Phoenix over the Summer.
#Therefore while the code refers to timezone as utc, the time is actually off by UTC-7 hours

clean_data <- raw_data |>
  mutate(Timestamp = Timestamp + 25200) |> #adding the 7 hour offset to the UTC time which is off in the device.
  mutate(Timestamp = anytime(Timestamp, tz = "America/Los_Angeles")) #reading in as the tiemzone for Phoenix

#Plot the airtemp data to test
clean_data |>
  ggplot() + geom_point(aes(x=Timestamp, y=AirTemp_C))

