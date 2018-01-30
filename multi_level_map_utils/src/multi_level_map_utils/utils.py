#!/usr/bin/env python

def construct(level_id, suffix):
  assert '/' not in level_id, "level id cannot contain forward slash"
  return level_id + '_' + suffix

def deconstruct(level_id, suffix):
  pass

def frameIdFromLevelId(level_id):
  return construct(level_id,'map')

def mapTopicFromLevelId(level_id):
  return construct(level_id,'map')

def metadataTopicFromLevelId(level_id):
  return construct(level_id,'map_metadata')

def mapServiceFromLevelId(level_id):
  return construct(level_id,'static_map')

def levelIdFromLevelFrameId(frame_id):
  assert frame_id[-4:] == '_map'
  return frame_id[:-4]

def levelIdFromLevelMapTopic(map_topic):
  return levelIdFromLevelFrameId(map_topic)
