from datetime import datetime
def getFileName(name:str):
    fileName = str(datetime.now())
    fileName = fileName.split(".")
    fileName = fileName[0].split(" ")
    temp = fileName[1].split(":")
    fileName[1] = f"{temp[0]}-{temp[1]}-{temp[2]}"
    fileName = f"{fileName[0]}_{fileName[1]}"
    return f"/home/inspiration/csvData/{fileName}_{name}_data.csv"