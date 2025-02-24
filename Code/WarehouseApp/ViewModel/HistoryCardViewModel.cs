using Models;
using WarehouseApp.ViewModel;

public class HistoryCardViewModel : ViewModelBase
{
    private readonly OrderItem linkedComponent;

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

    public string Title => linkedComponent.Goods.Name;

    public HistoryCardViewModel(OrderItem linkedComponent)
    {
        this.linkedComponent = linkedComponent;
    }
}