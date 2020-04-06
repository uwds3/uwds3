from pyuwds3.types.temporal_relation import TemporalRelation, Event


class Monitor(object):
    def __init__(self, simulator=None, beliefs_base=None):
        self.relations = []
        self.relations_index = {}
        self.simulator = None
        self.beliefs_base = None

    def trigger_action(self, subject, action, object=None):
        if object is not None:
            action = Event(subject.id, action, object=object.id)
            print(subject.id[:6] + " " + action + " " + object.id[:6])
        else:
            action = Event(subject.id, action)
            print(subject.id[:6] + " " + action)
        self.relations.append(action)

    def start_relation(self, subject, relation, object):
        r = TemporalRelation()
        r.start()
        self.relations.append(r)
        self.relations_index[subject.id+str(relation)+object.id] = len(self.relations)-1

    def end_relation(self, subject, relation, object):
        if subject.id+str(relation)+object.id in self.relations_index:
            self.relations[self.relations_index[subject.id+str(relation)+object.id]].end()
