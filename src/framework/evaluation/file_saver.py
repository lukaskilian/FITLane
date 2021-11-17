import json
import os
from abc import ABC, abstractmethod
from typing import Dict, Any

from framework.simulator.sensor_data import CameraImage
from framework.util import files, dictionaries


class FileSaver(ABC):

    def __init__(self, directory_name: str):
        self._directory_name = directory_name

    @abstractmethod
    def write_to_file(self, to_save_obj: Any, filename: str) -> None:
        """
        Speichert das Objekt in einem bestimmten Format als Datei ab
        :param to_save_obj: Objekt, welches gespeichert werden soll
        :param filename: Dateiname (ohne Ordnerpfad und Endung)
        """
        files.create_dir_if_not_exists(self._directory_name)


class JSONFileSaver(FileSaver):
    FILE_EXTENSION = "json"

    def write_to_file(self, to_save_obj: Any, filename: str) -> None:
        """
        Wandelt das Objekt in ein Dictionary um, speichert enthaltene Bilder, sowie das Objekt als JSON
        :param to_save_obj: Objekt, welches gespeichert werden soll
        :param filename: Dateiname (ohne Ordnerpfad und Endung)
        """
        super(JSONFileSaver, self).write_to_file(to_save_obj, filename)
        to_save_json = json.dumps(to_save_obj,
                                  default=lambda obj: self.__save_images_and_convert_to_dict(obj),
                                  indent=2)
        self.__write(filename, to_save_json)

    def __write(self, filename: str, text: str) -> None:
        filepath = os.path.join(self._directory_name, f"{filename}.{self.FILE_EXTENSION}")
        files.write(filepath, text)

    def __save_images_and_convert_to_dict(self, obj) -> Dict[str, Any]:
        if isinstance(obj, CameraImage):
            path = os.path.join(self._directory_name, "img")
            files.create_dir_if_not_exists(path)
            obj.save_to_disk(path)
        return dictionaries.convert_to_dict(obj)
