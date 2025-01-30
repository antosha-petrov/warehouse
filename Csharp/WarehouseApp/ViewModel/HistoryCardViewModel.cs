using WarehouseApp.ViewModel;

public class HistoryCardViewModel : ViewModelBase
{
    private readonly WarehouseApp.Model.Component linkedComponent;

    public int Rack
    {
        get => linkedComponent.Rack;
        set
        {
            linkedComponent.Rack = value;
            OnPropertyChanged();
        }
    }

    public int Cell
    {
        get => linkedComponent.Cell;
        set
        {
            linkedComponent.Cell = value;
            OnPropertyChanged();
        }
    }

    public int Shelf
    {
        get => linkedComponent.Shelf;
        set
        {
            linkedComponent.Shelf = value;
            OnPropertyChanged();
        }
    }

    public string Title => linkedComponent.Name;

    public HistoryCardViewModel(WarehouseApp.Model.Component linkedComponent)
    {
        this.linkedComponent = linkedComponent;
    }
}