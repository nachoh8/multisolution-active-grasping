class Metric(object):
    def __init__(self, res: any = None) -> None:
        self.res = res

    def get_name(self) -> str:
        raise Exception("This is an abstract class")
    
    # def set_result(self, res: any) -> None:
    #     self.res = res
    
    def get_metric_names(self) -> "list[str]":
        raise Exception("This is an abstract class")

    def get_metrics(self) -> "list[tuple[str, any]]":
        raise Exception("This is an abstract class")

    def get_failure(self) -> str:
        return ""

    def get_metadata(self) -> "list[tuple[str, any]]":
        return []

class BasicMetric(Metric):
    def __init__(self, res: float = None) -> None:
        super().__init__(res)

    def get_name(self) -> str:
        return "basic"
    
    # def set_result(self, res: float) -> None:
    #     self.res = res
    
    def get_metric_names(self) -> "list[str]":
        return ["outcome"]

    def get_metrics(self) -> "list[tuple[str, any]]":
        return [("outcome", self.res)]
