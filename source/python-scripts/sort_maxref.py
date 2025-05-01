#!/usr/bin/python3
import os
import re
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import ElementTree
from html.parser import HTMLParser

sort_elements = [
    ['inletlist', 'inlet', 'id'],
    ['outletlist', 'outletl', 'id'],
    ['methodlist', 'method', 'name'],
    ['jittermethodlist', 'jittermethod', 'name'],
    ['attributelist', 'attribute', 'name'],
    ['seealsolist', 'seealso', 'name']
]

class InlineMarkupParser(HTMLParser):
    def __init__(self, root_tag='discussion'):
        super().__init__()
        self.root = ET.Element(root_tag)
        self.stack = [self.root]

    def handle_starttag(self, tag, attrs):
        el = ET.SubElement(self.stack[-1], tag)
        self.stack.append(el)

    def handle_endtag(self, tag):
        if len(self.stack) > 1:
            self.stack.pop()

    def handle_data(self, data):
        parent = self.stack[-1]
        if parent.text is None and parent is not self.root:
            parent.text = data
        else:
            # If the last child exists, set its tail
            if len(parent):
                last = parent[-1]
                if last.tail is None:
                    last.tail = data
                else:
                    last.tail += data
            else:
                parent.text += data


def parse_inline_markup(markup: str, tag_name:str = "discussion") -> ET.Element:
    parser = InlineMarkupParser(tag_name)
    parser.feed(markup)
    return parser.root

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

def sort_ref_entries(tree: ElementTree, container_type: str = "methodlist", tag_type: str = "method",
                     sort_attr: str = "name") -> ElementTree:
    root = tree.getroot()

    container = root.find(container_type)
    if container is None:
        return tree

    tags = list(container.findall(tag_type))

    by_sort_attr = {}
    sort_keys = []
    for t in tags:
        by_sort_attr[t.attrib[sort_attr]] = t
        sort_keys.append(t.attrib[sort_attr])

    sorted_keys = natural_sort(sort_keys)

    tags.sort(key=lambda el: str(el.attrib[sort_attr]))

    for item in container.findall(tag_type):
        container.remove(item)

    for k in sorted_keys:
        container.append(by_sort_attr[k])
    return tree

def add_discussion(tree: ElementTree, text:str) -> ElementTree:
    root = tree.getroot()
    discussion = root.find('discussion')
    if discussion is not None:
        root.remove(discussion)
    discussion = parse_inline_markup(text)
    root.append(discussion)
    return tree

def print_file_list(files: list) -> list:
    selected_files = []
    i = 0
    for f in files:
        print(f"[{i}] {f}")
        i = i + 1
    print(f"[all] format all")
    print(f"[x] exit")

    print("Chose file to format:")
    choice = input().lower()
    if choice == 'all':
        selected_files = files
    elif choice == 'x':
        exit(0)
    else:
        try:
            file_index = int(choice)
            if file_index > len(files) - 1:
                raise ValueError()
        except ValueError:
            print("Invalid choice")
            return print_file_list(files)
        selected_files = [files[file_index]]
    return selected_files


def main():
    path = "../../docs"
    extra_content_path = '../projects'
    if not os.path.exists(path):
        print(f"{path} not found.")
        exit(0)
    files = os.listdir(path)
    files = [
        f for f in files
        if os.path.isfile(path + '/' + f) and f.endswith('maxref.xml')
    ]
    if len(files) == 0:
        print(f"No .maxref.xml files found in{path}.")
        exit(0)
    selected_files = print_file_list(files)
    for file in selected_files:
        tree = ET.parse("../../docs/" + file)
        for sort_element in sort_elements:
            tree = sort_ref_entries(tree, sort_element[0], sort_element[1], sort_element[2])

        discussion_text_path = extra_content_path + '/' +  file[:-11] + '/discussion.txt'
        discussion_content = ""
        if os.path.exists(discussion_text_path):
            with open(discussion_text_path, 'r', encoding='utf-8') as f:
                discussion_content = f.read()
        if discussion_content != "":
            tree = add_discussion(tree, discussion_content)
        out_path = "../../docs/" + file
        tree.write(out_path, encoding='utf-8', xml_declaration=True)
    exit(0)

if __name__ == '__main__':
    main()
