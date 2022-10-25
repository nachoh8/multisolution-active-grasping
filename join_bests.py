import sys
import glob

from active_grasping.datalog import DataLog

flog = sys.argv[1]
fres = sys.argv[2]

print("Loading results from " + flog + " folder")
print("Save best results to " + fres + " file")

logs = glob.glob(flog + "/*.json")
print("Num. log files: " + str(len(logs)))

logger = DataLog(log_file=logs[0])
logger.load_json()

logger.log_grasps([])

for file in logs[1:]:
    _logger = DataLog(log_file=file)
    _logger.load_json()

    logger.best_results += _logger.best_results

logger.log_best_results(logger.best_results)

print("Total best grasps: " + str(len(logger.best_results)))
logger.save_json(json_file=fres)
