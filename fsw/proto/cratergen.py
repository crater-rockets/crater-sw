#!/bin/env python3

from pydantic_xml import BaseXmlModel, attr, element, wrapped
from pydantic import ConfigDict, conint
from typing import Annotated
from pathlib import Path
import click
from jinja2 import Environment, FileSystemLoader, select_autoescape


class Wip(BaseXmlModel):
    pass


class MavlinkDeprecated(BaseXmlModel):
    since: str = attr()
    replaced_by: str = attr()
    reason: str


class MavlinkEnumParam(BaseXmlModel):
    index: int = attr()
    label: str | None = attr(default=None)
    units: str | None = attr(default=None)
    enum: str | None = attr(default=None)
    decimalPlaces: int | None = attr(default=None)
    increment: int | None = attr(default=None)
    minValue: int | None = attr(default=None)
    maxValue: int | None = attr(default=None)
    multiplier: str | None = attr(default=None)
    reserved: bool = attr(default=False)
    default: str | None = attr(default=None)

    description: str


class MavlinkEnumEntry(BaseXmlModel):
    wip: Wip | None = element(tag="wip", default=None)
    deprecated: MavlinkDeprecated | None = element(tag="deprecated", default=None)

    name: str = attr()
    hasLocation: bool = attr(default=False)
    isDestination: bool = attr(default=False)
    missionOnly: bool = attr(default=False)

    value: Annotated[int, conint(ge=0, lt=256)] = attr()
    description: str = element(default="")
    params: list[MavlinkEnumParam] | None = element(tag="param", default=None)


class MavlinkEnum(BaseXmlModel):
    name: str = attr()
    bitmask: bool = attr(default=False)
    description: str = element(default="")

    entries: list[MavlinkEnumEntry] = element(tag="entry", default=[])


class MavlinkMessageField(BaseXmlModel):
    dtype: str = attr(name="type")
    name: str = attr()
    enum: str | None = attr(default=None)
    units: str | None = attr(default=None)
    multiplier: str | None = attr(default=None)
    display: str | None = attr(default=None)
    optional: str | None = attr(default=None)
    default: str | None = attr(default=None)
    increment: int | None = attr(default=None)
    minValue: int | None = attr(default=None)
    maxValue: int | None = attr(default=None)
    instance: bool = attr(default=False)
    invalid: str | None = attr(default=None)

    description: str


class MavlinkMessage(BaseXmlModel):
    id: int = attr()
    name: str = attr()

    wip: Wip | None = element(tag="wip", default=None)
    description: str = element(default="")
    fields: list[MavlinkMessageField] = element(tag="field", default=[])


class Mavlink(BaseXmlModel, tag="mavlink"):
    # model_config = ConfigDict(extra="forbid")

    varsion: int = element(tag="version")
    enums: list[MavlinkEnum] = wrapped("enums", element(tag="enum", default=[]))
    messages: list[MavlinkMessage] = wrapped(
        "messages", element(tag="message", default=[])
    )


@click.command("cratergen")
@click.argument("mavlink", type=click.Path(exists=True, dir_okay=False))
@click.option(
    "-t", "--template", type=click.Path(exists=True, dir_okay=True, file_okay=False)
)
@click.option("-o", "--output", type=click.Path(dir_okay=True))
def cratergen(template, mavlink, output):
    xml_doc = Path(mavlink).read_text()
    mavlink_data = Mavlink.from_xml(xml_doc)

    enums = {e.name: e for e in mavlink_data.enums}
    messages = {m.name: m for m in mavlink_data.messages}

    template_dir = Path(template)

    env = Environment(
        loader=FileSystemLoader(template_dir), autoescape=select_autoescape()
    )

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)

    for template_file in template_dir.glob("*.jinja"):
        template = env.get_template(template_file.name)
        with open(output / template_file.stem, "w") as f:
            f.write(template.render(enums=enums, messages=messages))


if __name__ == "__main__":
    cratergen()
