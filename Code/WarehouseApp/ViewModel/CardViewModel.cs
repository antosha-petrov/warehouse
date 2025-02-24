using System.Windows.Input;
using WarehouseApp.ViewModel;
using Models; 

public class CardViewModel : ViewModelBase
{
    private readonly OrderItem linkedComponent;

    public int Quantity 
    {
        get => linkedComponent.Quantity; 
        set
        {
            linkedComponent.Quantity = value;
            OnPropertyChanged();
        }
    }

    public string ImageSource => linkedComponent.Goods.ImageSource;

    public string Title => linkedComponent.Goods.Name;

    public CardViewModel(OrderItem linkedComponent)
    {
        this.linkedComponent = linkedComponent;

        PlusCommand = new Command(() => Quantity++);
        MinusCommand = new Command(() => 
        { 
            if (Quantity > 0) 
                Quantity--; 
        });
    }

    public ICommand PlusCommand { get; }

    public ICommand MinusCommand { get; }
}