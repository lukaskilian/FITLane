"""Utility-Script, welches Daten in eine Datei schreibt"""
import os


def create_dir_if_not_exists(dir_name: str) -> None:
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)


def write(filename: str, text: str) -> None:
    with open(filename, 'w', encoding='utf-8') as file:
        file.write(text)


def read(filename: str) -> str:
    with open(filename, 'r', encoding='utf-8') as file:
        return file.read()
