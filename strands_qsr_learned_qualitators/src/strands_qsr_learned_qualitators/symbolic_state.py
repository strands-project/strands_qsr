class SymbolicState(object):
    def __init__(self, scene_id):
        self._clauses = []
        
        self._scene_id = scene_id
        
    def add_clause(self, clause):
        self._clauses.append(clause)
        
    def __iter__(self):
        return iter(self._clauses)
    
    def __len__(self):
        return len(self._clauses)