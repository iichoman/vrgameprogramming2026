#pragma once
#include "CoreMinimal.h"
#include "FactoryLayoutModel.h"

/** Value snapshots only. A gesture is one transaction; cameras and actors never enter history. */
class FFactoryEditHistory
{
public:
    struct FSnapshot { FFactoryLayout Layout; FString Selection; };
    void Begin(const FFactoryLayout& Layout, const FString& Selection)
    { if (!Pending.IsSet()) Pending = FSnapshot{Layout, Selection}; }
    bool Commit(const FFactoryLayout& Layout)
    {
        if (!Pending.IsSet()) return false;
        const bool Changed = Pending->Layout.ToJson() != Layout.ToJson();
        if (Changed) { UndoStack.Add(*Pending); if (UndoStack.Num() > 100) UndoStack.RemoveAt(0); RedoStack.Reset(); }
        Pending.Reset(); return Changed;
    }
    bool Cancel(FFactoryLayout& Layout, FString& Selection)
    { if (!Pending.IsSet()) return false; Layout=Pending->Layout; Selection=Pending->Selection; Pending.Reset(); return true; }
    bool Undo(FFactoryLayout& Layout, FString& Selection) { return Restore(UndoStack, RedoStack, Layout, Selection); }
    bool Redo(FFactoryLayout& Layout, FString& Selection) { return Restore(RedoStack, UndoStack, Layout, Selection); }
    bool CanUndo() const { return !UndoStack.IsEmpty(); }
    bool CanRedo() const { return !RedoStack.IsEmpty(); }
    bool InTransaction() const { return Pending.IsSet(); }
    void Clear() { UndoStack.Reset(); RedoStack.Reset(); Pending.Reset(); }
private:
    bool Restore(TArray<FSnapshot>& From, TArray<FSnapshot>& To, FFactoryLayout& Layout, FString& Selection)
    {
        if (Pending.IsSet() || From.IsEmpty()) return false;
        To.Add({Layout,Selection}); FSnapshot Snapshot=From.Pop();
        Layout=MoveTemp(Snapshot.Layout); Selection=MoveTemp(Snapshot.Selection); return true;
    }
    TOptional<FSnapshot> Pending;
    TArray<FSnapshot> UndoStack, RedoStack;
};
