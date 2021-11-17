"""Skript, um Objekte in Dictionaries umzuwandeln"""
import logging
from typing import Dict, Any


def convert_to_dict(obj) -> Dict[str, Any]:
    """
    Wandelt ein beliebiges Objekt in ein Python-Dictionary um. Falls eine Klasse die Methode 'to_dict()' implementiert,
    wird diese genutzt. Ansonsten wird die Python-interne vars-Methode genutzt.
    :param obj: das umzuwandelnde Objekt
    :return: Dictionary, bei der die keys die Namen der Attribute und die Values die Attribut-Werte selbst sind (können
    wieder Objekte sein, dann muss diese Methode erneut aufgerufen werden.)
    """
    try:
        if callable(getattr(obj, "to_dict", None)):
            return obj.to_dict()
        else:
            return vars(obj)
    except Exception as e:
        logging.critical(f"Konnte {type(obj)} nicht serialisieren")
        raise e
